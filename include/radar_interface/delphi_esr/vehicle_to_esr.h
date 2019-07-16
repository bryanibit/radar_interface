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

#include "geometry_msgs/TwistStamped.h"
#include "radar_interface/can_tools.h"
#include "socketcan_bridge/topic_to_socketcan.h"
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <ros/ros.h>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/socketcan.h>

#define RAD_TO_DEG 57.295779513
#define ESR_MAX_TRACKS 64
#define ESR_GROUPING_MODE                                                      \
  0 // 0 - No grouping; 1 - moving only; 2 - stationary only; 3 - all
#define ESR_VEHICLE_INFO_MSG_ID_TX_1 0x4F0
#define ESR_VEHICLE_INFO_MSG_ID_TX_2 0x4F1

/*can_tools::CANParseValueInfo VEH_VEL = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 0,
    .END_BYTE = 1,
    .SHIFT = 5,
    .SCALE = 0.0625,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 127.9375,
    .MASK = {0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo VEH_VEL_DIR = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 1,
    .END_BYTE = 1,
    .SHIFT = 4,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

can_tools::CANParseValueInfo VEH_YAW_RATE = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 1,
    .END_BYTE = 2,
    .SHIFT = 0,
    .SCALE = 0.0625,
    .OFFSET = 0,
    .MIN = -128,
    .MAX = 127.9375,
    .MASK = {0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo VEH_YAW_RATE_VALID = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 3,
    .END_BYTE = 3,
    .SHIFT = 7,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00}};*/

can_tools::CANParseInfo VEH_YAW_RATE(11,      // start_bit
                                     12,      // length
                                     true,    // is_signed
                                     0.0625,  // scale
                                     0.0,     // offset
                                     -128,    // min
                                     127.9375 // max
);
can_tools::CANParseInfo VEH_YAW_RATE_VALID(31,    // start_bit
                                           1,     // length
                                           false, // is_signed
                                           1,     // scale
                                           0.0,   // offset
                                           0,     // min
                                           1      // max
);
can_tools::CANParseInfo VEH_VEL(7,       // start_bit
                                11,      // length
                                false,   // is_signed
                                0.0625,  // scale
                                0.0,     // offset
                                -128,    // min
                                127.9375 // max
);
can_tools::CANParseInfo VEH_VEL_DIR(12,    // start_bit
                                    1,     // length
                                    false, // is_signed
                                    1,     // scale
                                    0.0,   // offset
                                    0,     // min
                                    1      // max
);
can_tools::CANParseInfo VEH_VEL_VALID(61,    // start_bit
                                      1,     // length
                                      false, // is_signed
                                      1,     // scale
                                      0.0,   // offset
                                      0,     // min
                                      1      // max
);
can_tools::CANParseInfo MAX_TRACKS(53,    // start_bit
                                   6,     // length
                                   false, // is_signed
                                   1,     // scale
                                   1,     // offset
                                   1,     // min
                                   64     // max
);
can_tools::CANParseInfo TO_RADIATE(55,    // start_bit
                                   1,     // length
                                   false, // is_signed
                                   1,     // scale
                                   0,     // offset
                                   0,     // min
                                   1      // max
);
can_tools::CANParseInfo GROUPING_MODE(59,    // start_bit
                                      2,     // length
                                      false, // is_signed
                                      1,     // scale
                                      0,     // offset
                                      0,     // min
                                      3      // max
);

class VehicleToESR : public socketcan_bridge::TopicToSocketCAN {
public:
  VehicleToESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
               can::DriverInterfaceSharedPtr driver);
  void sendCanFrame(const ros::TimerEvent &event);
  virtual ~VehicleToESR();

private:
  ros::Subscriber twist_topic_;
  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void setFrameProperties(can::Frame *frame);
  geometry_msgs::TwistStamped twist_;
  can::DriverInterfaceSharedPtr driver_interface_;
  can::Frame frame_vehicle1_;
  can::Frame frame_vehicle2_;
};