#include "gf3_hardware/mecanum_control.hpp"
namespace gf3_hardware
{
  const double WHEEL_RADIUS = 0.05;
  const double WHEEL_SEPARATION_WIDTH = 0.405;
  const double WHEEL_SEPARATION_LENGTH = 0.304;
  const double ratio = 10.0;
  const uint32_t motor_id_[2] = {0x001U, 0x002U}; // (FL - 0x002, M1), (RL - 0x002, M2), (FR - 0x001, M1), (RR - 0x001, M2)

MecanumControl::MecanumControl()
:Node("MecanumControl")
{
  can_state_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_feedback", 1, std::bind(&MecanumControl::can_reply_callback, this, _1));
  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&MecanumControl::cmd_vel_callback, this, _1));
  can_command_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_command", 10);
  odom_publisher_ = this->create_publisher<can_msgs::msg::Frame>("odom", 10);
  
  
}

MecanumControl::~MecanumControl()
{
  
}

void MecanumControl::cmd_vel_callback(const geometry_msgs::msg::Twist &msg){
  
  // double wheel_front_left = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z); 
  double wheel_front_left = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);
  double wheel_front_right = (1/WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);
  double wheel_rear_left = (1/WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);
  double wheel_rear_right = (1/WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*msg.angular.z);

  int16_t target_FL_rpm = wheel_front_left / (2 * M_PI) * 60 * ratio;
  int16_t target_FR_rpm = wheel_front_right / (2 * M_PI) * 60 * ratio;
  int16_t target_RL_rpm = wheel_rear_left / (2 * M_PI) * 60 * ratio;
  int16_t target_RR_rpm = wheel_rear_right / (2 * M_PI) * 60 * ratio;
  //   int16_t target_FL_rpm =10;
  // int16_t target_FR_rpm = 10;
  // int16_t target_RL_rpm = 10;
  // int16_t target_RR_rpm = 10;

  can_msgs::msg::Frame command;
  command.data[0] = 0xCF;
  command.data[1] = 0x01; // M1 enable
  command.data[4] = 0x01; // M2 enable
  command.data[7] = 0x07; // set reply type

  for (uint8_t idx = 0 ;idx < 2; idx++){
      
      if (idx == 1){ // FL, RL
        command.id = motor_id_[idx];
        command.data[2] = target_FL_rpm & 0xFF; // for FL(M1)
        command.data[3] = (target_FL_rpm >> 8) & 0xFF;
        command.data[5] = target_RL_rpm & 0xFF; // for RL(M2)
        command.data[6] = (target_RL_rpm >> 8) & 0xFF;
      }
      if (idx == 0){ // FR, RR
        command.id = motor_id_[idx];
        command.data[2] = target_FR_rpm & 0xFF; // for FR(M1)
        command.data[3] = (target_FR_rpm >> 8) & 0xFF;
        command.data[5] = target_RR_rpm & 0xFF; // for RR(M2)
        command.data[6] = (target_RR_rpm >> 8) & 0xFF;
      }
      can_command_publisher_->publish(command);
  }
}

void MecanumControl::can_reply_callback(const can_msgs::msg::Frame &msg)
{
  auto it = std::find(std::begin(motor_id_), std::end(motor_id_), msg.id);
  if (it != std::end(motor_id_)) {
      int index = std::distance(motor_id_, it);
      std::cout << "Reply from Index: " << index << "found" << std::endl;
  } else {
      RCLCPP_INFO(rclcpp::get_logger("MecanumControl"), "not found...");

    }
  // c4(196)-motor1, c9(201)-motor2 
  if (msg.id == 0x001U){ // Right controller
    std::cout <<"ID 1 received" <<std::endl;
    if (msg.data[0] == 0xC4U){ // FR

    }
    if (msg.data[0] == 0xC9U){ // RR

    }
  }
  else if(msg.id == 0x002U){ // Left controller
    std::cout <<"ID 2 received" <<std::endl;
    if (msg.data[0] == 0xC4U){ // FL

    }
    if (msg.data[0] == 0xC9U){ // RL

    }
  }
  int16_t rpm = msg.data[2] + ((msg.data[3] << 8) & 0xFF'00);
 
  int16_t received_FL_rpm_ = wheel_front_left / (2 * M_PI) * 60 * ratio;
  int16_t received_FR_rpm_ = wheel_front_right / (2 * M_PI) * 60 * ratio;
  int16_t received_RL_rpm_ = wheel_rear_left / (2 * M_PI) * 60 * ratio;
  int16_t received_RR_rpm_ = wheel_rear_right / (2 * M_PI) * 60 * ratio;
  
}


int MecanumControl::MainLoop()
{
  rclcpp::spin_some(shared_from_this());
  return 0;
}
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto mecanum = std::make_shared<gf3_hardware::MecanumControl>();
  while(rclcpp::ok()){
    mecanum->MainLoop();
  }
  rclcpp::shutdown();
  return 0;
}