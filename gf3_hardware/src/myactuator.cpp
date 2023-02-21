#include "gf3_hardware/myactuator.hpp"

namespace gf3_hardware
{
    can_msgs::msg::Frame Myactuator::getPosition(uint32_t const& motorId)
    {
        auto frame = can_msgs::msg::Frame();
        frame.id = motorId;
        frame.data[0] = Commands::Myactuator::READ_MULTITURN_ANGLE;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        return frame;
    }

    can_msgs::msg::Frame Myactuator::setPosition(uint32_t const& motorId, std::array<uint8_t, 8UL> data)
    {
        auto frame = can_msgs::msg::Frame();
        frame.id = motorId;
        frame.data = data;
        // frame.data[0] = Commands::Myactuator::SET_POS_COMMAND;
        // frame.data[1] = 0x00;
        // frame.data[2] = 0x00;
        // frame.data[3] = 0x00;
        // frame.data[4] = 0x00;
        // frame.data[5] = 0x00;
        // frame.data[6] = 0x00;
        // frame.data[7] = 0x00;

        return frame;
    }
}