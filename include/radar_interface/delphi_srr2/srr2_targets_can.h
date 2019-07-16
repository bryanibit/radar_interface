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

#ifndef SRR2_CAN_INTERFACE_H
#define SRR2_CAN_INTERFACE_H

#include "radar_interface/RadarTargetArray.h"
#include "radar_interface/can_tools.h"
#include <can_msgs/Frame.h>
#include <ros/ros.h>
// #include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/string.h>
#include <string.h>

#define SRR2_MAX_TARGET_NUMBER 64
#define SRR2_LEFT_TARGET_START 0x500
#define SRR2_LEFT_TARGET_END 0x53F
#define SRR2_LEFT_STATUS1 0x4E0
#define SRR2_RIGHT_TARGET_START 0x600
#define SRR2_RIGHT_TARGET_END 0x63F
#define SRR2_RIGHT_STATUS1 0x4D0
#define SRR2_LEFT 1
#define SRR2_RIGHT 2
#define SRR2_BOTH 3
#define DEG_TO_RAD 0.017453293

namespace radar_interface {
class CANInterfaceSRR2 {
public:
  CANInterfaceSRR2(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                   can::DriverInterfaceSharedPtr driver, std::string radar_name,
                   int left_right_both);

private:
  void frameCallback(const can::Frame &f);
  void stateCallback(const can::State &s);
  void aggregateTargets(const can::Frame &f, bool is_left);
  void parseTarget(const can::Frame &f, bool is_left);
  ros::Publisher can_topic_;
  ros::Publisher target_array_topic_left_;
  ros::Publisher target_array_topic_right_;
  can::DriverInterfaceSharedPtr driver_;

  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  radar_interface::RadarTargetArray targets_msg_left_;
  radar_interface::RadarTargetArray targets_msg_right_;
  bool first_target_arrived_left_;
  bool last_target_arrived_left_;
  bool first_target_arrived_right_;
  bool last_target_arrived_right_;
  int target_count_left_;
  int target_count_right_;
  bool use_left_;
  bool use_right_;
  std::string radar_name_;
  std::string radar_name_left_;
  std::string radar_name_right_;
  double timestamp_left_;
  double timestamp_right_;
  double absolute_timestamp_right_;
  double absolute_timestamp_left_;
  bool first_timestamp_received_left_;
  bool first_timestamp_received_right_;
  bool use_dsp_timestamps_;
};

void convertSocketCANToMessage(const can::Frame &f, can_msgs::Frame &m) {
  m.id = f.id;
  m.dlc = f.dlc;
  m.is_error = f.is_error;
  m.is_rtr = f.is_rtr;
  m.is_extended = f.is_extended;

  for (int i = 0; i < 8; i++) // always copy all data, regardless of dlc.
  {
    m.data[i] = f.data[i];
  }
};

// CAN messages layout
can_tools::CANParseInfo SRR2_TARGET_VALID_LEVEL(7,     // start_bit
                                                3,     // length
                                                false, // is_signed
                                                1,     // scale
                                                0.0,   // offset
                                                0,     // min
                                                7      // max
);
can_tools::CANParseInfo SRR2_TARGET_STATUS(4,     // start_bit
                                           1,     // length
                                           false, // is_signed
                                           1,     // scale
                                           0.0,   // offset
                                           0,     // min
                                           1      // max
);
can_tools::CANParseInfo SRR2_TARGET_AMPLITUDE(3,     // start_bit
                                              12,    // length
                                              true,  // is_signed
                                              0.125, // scale
                                              0.0,   // offset
                                              -24,   // min
                                              40     // max
);
can_tools::CANParseInfo SRR2_TARGET_ANGLE(23,        // start_bit
                                          16,        // length
                                          true,      // is_signed
                                          0.0078125, // scale
                                          0.0,       // offset
                                          -102.4,    // min
                                          102.2      // max
);
can_tools::CANParseInfo SRR2_TARGET_RANGE(39,        // start_bit
                                          16,        // length
                                          false,     // is_signed
                                          0.0078125, // scale
                                          0.0,       // offset
                                          0,         // min
                                          204.7      // max
);
can_tools::CANParseInfo SRR2_TARGET_RANGE_RATE(55,        // start_bit
                                               16,        // length
                                               true,      // is_signed
                                               0.0078125, // scale
                                               0.0,       // offset
                                               -81.92,    // min
                                               81.91      // max
);
can_tools::CANParseInfo SRR2_TIMESTAMP(7,     // start_bit
                                       16,    // length
                                       false, // is_signed
                                       0.001, // scale
                                       0.0,   // offset
                                       0,     // min
                                       655536 // max
);

}; // namespace radar_interface

#endif // SRR2_CAN_INTERFACE_H