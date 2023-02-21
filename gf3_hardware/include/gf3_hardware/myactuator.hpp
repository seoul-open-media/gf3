#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "can_msgs/msg/frame.hpp"

namespace gf3_hardware::Commands::Myactuator
{
    uint8_t const READ_POS_KP = 0x30;
    uint8_t const READ_POS_KI = 0x31;
    uint8_t const READ_VEL_KP = 0x32;
    uint8_t const READ_VEL_KI = 0x33;
    uint8_t const READ_TORQUE_KP = 0x34;

    // uint8_t const READ_TORQUE_KI = 0x35;
    // uint8_t const WRITE_POS_KP_RAM = 0x36;
    // uint8_t const WRITE_POS_KI_RAM = 0x37;
    // uint8_t const WRITE_VEL_KP_RAM = 0x38;
    // uint8_t const WRITE_VEL_KI_RAM = 0x39;

    uint8_t const WRITE_TORQUE_KP_RAM = 0x3A;
    uint8_t const WRITE_TORQUE_KI_RAM = 0x3B;
    uint8_t const WRITE_POS_KP_ROM = 0x3C;
    uint8_t const WRITE_POS_KI_ROM = 0x3D;
    uint8_t const WRITE_VEL_KP_ROM = 0x3E;
    uint8_t const WRITE_VEL_KI_ROM = 0x3F;
    uint8_t const WRITE_TORQUE_KP_ROM = 0x40;
    uint8_t const WRITE_TORQUE_KI_ROM = 0x41;
    uint8_t const READ_ACCEL = 0x42;
    uint8_t const WRITE_ACCEL = 0x43;
    uint8_t const READ_MULTITURN_POS = 0x60;
    uint8_t const READ_ORIGINAL_POS = 0x61;
    uint8_t const READ_MULTITURN_OFFSET = 0x62;
    uint8_t const WRITE_ENCODER_ZERO = 0x63;
    uint8_t const WRITE_ENCODER_CURRENT_POS_AS_ZERO = 0x64;
    uint8_t const READ_MULTITURN_ANGLE = 0x92;
    uint8_t const READ_SINGLE_CIRCLE_ANGLE = 0x94;

    uint8_t const READ_MOTOR_STATUS1 = 0x9A;
    uint8_t const READ_MOTOR_STATUS2 = 0x9C;
    uint8_t const READ_MOTOR_STATUS3 = 0x9D;

    uint8_t const SHUTDOWN = 0x80;
    uint8_t const STOP = 0x81;
    uint8_t const ENABLE = 0x88;

    uint8_t const TORQUE_COMMAND = 0xA1;
    uint8_t const SPEED_COMMAND = 0xA2;
    uint8_t const SET_POS_COMMAND = 0xA3;
    uint8_t const ABS_POS_COMMAND = 0xA4;
    uint8_t const REL_POS_COMMAND = 0xA8;


    uint8_t const READ_OPERATING_MODE = 0x70;
    uint8_t const READ_MOTOR_POWER = 0x71;
    uint8_t const READ_AUXILIARY_VOLTAGE = 0x72;
    uint8_t const WRITE_TORQUE_FEEDFORWARD = 0x73;

    uint8_t const RESET = 0x76;
    uint8_t const BRAKE_RELEASE = 0x77;
    uint8_t const BRAKE_LOCK = 0x78;
    uint8_t const CAN_ID_SETUP = 0x79;
    uint8_t const READ_RUNTIME = 0xB1;
    uint8_t const READ_SOFTWARE_VERSION = 0xB2;
    uint8_t const COMM_INTERRUPT_TIMEOUT = 0xB3;
}

namespace gf3_hardware
{
class Myactuator{
    public:
    
        /// \brief Reset a given motor
        /// \param[in] motorId Motor id
        void reset(uint32_t const& motorId);

        void enable(uint32_t const& motorId);
        void disable(uint32_t const& motorId);

        /// \brief Get the motor acceleration command
        /// \param[in] motorId Motor id
        /// \return Acceleration command, in dps/s^2 ; 0 on failure.
        int32_t getAccelerationCommand(uint32_t const& motorId);

        /// \brief Get the motor position (multiturn)
        /// \param[in] motorId Motor id
        /// \return Position,  command, in dps/s ; -1 on failure.
        can_msgs::msg::Frame getPosition(uint32_t const& motorId);

        can_msgs::msg::Frame setPosition(uint32_t const& motorId, std::array<uint8_t, 8UL> data);

        /// \brief Close-loop speed contro: set motor target speed
        /// \param[in] motorId Motor id
        /// \param[in] targetSpeed Target speed, 0.01dps/LSB
        /// \return Current speed, 0 on failure.
        int setSpeed(uint32_t const& motorId, int32_t const& targetSpeed);

        /// \brief Torque control: set motor torque (i.e. quadrature current) target
        /// \param[in] motorId Motor id
        /// \param[in] targetTorque Target torque, 0.01A/LSB
        /// \return Current torque (current), 0.01A/LSB ; 0 on failure.
        int setTorque(uint32_t const& motorId, int32_t const& targetTorque);

    private:

        /// \brief Send and recieve a data frame from the motor.
        ///
        /// \param[in, out] message The message to send, modified in place.
        /// \param waitForReply Whether or not to wait for a reply
        /// \return 0 on succes ; -1 if write failed ; -2 if write succeeded but read failed.
        // int canReadWrite(CANMessage& message, bool const& waitForReply = true);

};
}