
/** @file ros2socketcan.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg
 *  @brief  bidirectional ROS2 to CAN interface with topics and service
 */

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "std_msgs/msg/int32.hpp"
#include "can_msgs/msg/frame.hpp"
#include <memory>
// #include "can_msgs/msg/frame.hpp"
#include <cstdio>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "log.h"