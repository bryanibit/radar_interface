/**
 *  This file is a part of radar_interface.
 *
 *  Copyright (C) 2018 Juraj Persic, University of Zagreb Faculty of Electrical
 Engineering and Computing

 *  radar_interface is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "geometry_msgs/Twist.h"
#include "radar_interface/can_tools.h"
#include "socketcan_bridge/topic_to_socketcan.h"
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <ros/ros.h>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/socketcan.h>

// can_tools::CANParseValueInfo VEH_VEL = {
//     .MSG_ID = 0x20,
//     .START_BYTE = 5,
//     .END_BYTE = 6,
//     .SHIFT = 0,
//     .SCALE = 0.01,
//     .OFFSET = 0,
//     .MIN=0,
//     .MAX=655.35,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00}};
// can_tools::CANParseValueInfo VEH_VEL_UB = {
//     .MSG_ID = 0x20,
//     .START_BYTE = 7,
//     .END_BYTE = 7,
//     .SHIFT = 4,
//     .SCALE = 1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 1,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08}};
can_tools::CANParseValueInfo VEH_VEL = {
    .MSG_ID = 0x65,
    .START_BYTE = 6,
    .END_BYTE = 7,
    .SHIFT = 0,
    .SCALE = 0.01,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 320,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff}};
can_tools::CANParseValueInfo VEH_VEL_UB = {
    .MSG_ID = 0x65,
    .START_BYTE = 5,
    .END_BYTE = 5,
    .SHIFT = 0,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}};
can_tools::CANParseValueInfo VEH_VEL_QF = {
    .MSG_ID = 0x65,
    .START_BYTE = 5,
    .END_BYTE = 5,
    .SHIFT = 1,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 3,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00}};

can_tools::CANParseValueInfo VEH_YAW_RATE = {
    .MSG_ID = 0x100,
    .START_BYTE = 6,
    .END_BYTE = 7,
    .SHIFT = 0,
    .SCALE = 0.01221,
    .OFFSET = -100,
    .MIN = -100,
    .MAX = 100.036,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff}};

// const unsigned short int VEH_YAW_RATE_MSG_ID = 0x100;
// const unsigned short int VEH_YAW_RATE_START_BYTE = 6;
// const unsigned short int VEH_YAW_RATE_SHIFT = 0;
// const unsigned short int VEH_YAW_RATE_SCALE = 0.0221;
// const unsigned short int VEH_YAW_RATE_OFFSET = -100;
// const char VEH_YAW_RATE_MASK[2] = {0x0f, 0xff};

class VehicleToSRR2 : public socketcan_bridge::TopicToSocketCAN {
public:
  VehicleToSRR2(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                can::DriverInterfaceSharedPtr driver);
  void sendCanFrame(const ros::TimerEvent &event);
  virtual ~VehicleToSRR2();

private:
  ros::Subscriber twist_topic_;
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void setFrameProperties(can::Frame *frame);
  geometry_msgs::Twist twist_;
  can::DriverInterfaceSharedPtr driver_interface_;
  can::Frame frame_vel;
  can::Frame frame_yaw_rate;
};
