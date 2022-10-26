
#include <connect_robot.hpp>
#include <RMDX.hpp>

class ConnectCanClass
{
  public:
    int InitCanInterface(const char *ifname)

  private:

};
/**
 * @brief CAN 인터페이스를 초기화한다.
 * @param[in] ifname CAN 인터페이스 이름
 * @retval 양수: CAN 소켓 디스크립터
 * @retval -1: 실패
 */  
int InitCanInterface(const char *ifname)
{
  /* 
   * CAN 소켓을 생성한다.
   */
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock == -1) {
  printf("Fail to create can socket for %s - %m\n", ifname);
    return -1;
  }
  printf("Success to create can socket for %s\n", ifname);

  /* 
   * CAN 인터페이스 식별번호를 획득한다.
   */
  struct ifreq ifr;
  strcpy(ifr.ifr_name, ifname);
  int ret = ioctl(sock, SIOCGIFINDEX, &ifr);
  if (ret == -1) {
    perror("Fail to get can interface index -");
    return -1;
  }
  printf("Success to get can interface index: %d\n", ifr.ifr_ifindex);

  /*
   * CAN 소켓을 바인딩한다.
   */
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret == -1) {
    perror("Fail to bind can socket -");
    return -1;
  }
  printf("Success to bind can socket\n");

  return sock;
}

/**
 * @brief CAN 프레임을 전송한다.
 * @param[in] sock CAN 소켓 디스크립터
 * @param[in] id CAN id (11비트 또는 29비트 길이)
 * @param[in] data 전송할 CAN 프레임 데이터
 * @param[in] data_len 전송할 CAN 프레임 데이터의 길이
 * @retval 0: 성공
 * @retval -1: 실패
 */ 
int TransmitCanFrame(const int sock, const uint32_t id, const uint8_t *data, const size_t data_len)
{
  /*
   * 전송할 CAN 프레임을 설정한다.
   */
  struct can_frame frame;
  frame.can_id = id & 0x1fffffff;
  frame.can_id |= (1 << 31);
  memcpy(frame.data, data, data_len);
  frame.can_dlc = data_len;

  /*
   * 전송한다.
   */
  int tx_bytes = write(sock, &frame, sizeof(frame));
  if (tx_bytes == -1) {
    perror("Fail to transmit can frame -");
    return -1;
  } 
  printf("Success to transmit can frame - %d bytes is transmitted\n", tx_bytes);
  return 0;
}


/// CAN 프레임 최대 길이
#define CAN_FRAME_MAX_LEN 8 

/**
 * @brief CAN 프레임을 수신한다.
 * @param[in] 프레임을 수신한 CAN 소켓 디스크립터
 * @retval 0: 성공
 * @retval -1: 실패
 */ 
int ReceiveCanFrame(const int sock)
{
  /*
   * CAN 프레임을 수신한다.
   */ 
  struct can_frame frame;
  int rx_bytes = read(sock, &frame, sizeof(frame));
  if (rx_bytes < 0) {
    perror("Fail to receive can frame - ");
    return -1;
  } else if (rx_bytes < (int)sizeof(struct can_frame)) {
    printf("Incomplete can frame is received - rx_bytes: %d\n", rx_bytes);
    return -1;
  } else if (frame.can_dlc > CAN_FRAME_MAX_LEN) {
    printf("Invalid dlc: %u\n", frame.can_dlc);
    return -1;
  }

  /*
   * 프레임 유형에 따라 처리한다.
   */
  if (((frame.can_id >> 29) & 1) == 1) {
    printf("Error frame is received\n");
  } else if (((frame.can_id >> 30) & 1) == 1) {
    printf("RTR frame is received\n");
  } else {
    if (((frame.can_id >> 31) & 1) == 1) {
      printf("11bit long std can frame is received\n");
    } else {
      printf("29bit long ext can frame is received\n");
    }
    
    // TODO: 프레임 처리
  }

  return 0;
}


/// CAN ID 
#define CAN_ID 0x141

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  const uint8_t READ_POS_KP = 0x30;
  printf("hello world gf3_bringup package\n");
  /*
   * CAN 인터페이스를 초기화한다.
   */
  int sock = InitCanInterface("can0");
  if (sock < 0) {
    return -1;
  }

  /*
   * CAN 데이터 프레임을 송신한다.
   */ 
  rclcpp::init(argc, argv);
  rclcpp::Rate loop_rate(10);
  while(rclcpp::ok())
  {
    uint8_t can_data[CAN_FRAME_MAX_LEN] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
    TransmitCanFrame(sock, CAN_ID, can_data, sizeof(can_data));

    /*
    * CAN 프레임을 수신한다. (별도의 수신 쓰레드 내에서 호출 가능)
    */  
    // ReceiveCanFrame(sock);
    loop_rate.sleep();
  }

  return 0;
}
