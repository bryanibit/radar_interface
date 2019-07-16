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

#ifndef ESR_CAN_INTERFACE_H
#define ESR_CAN_INTERFACE_H

#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/VehicleInfo.h"
#include "radar_interface/can_tools.h"
#include <can_msgs/Frame.h>
#include <ros/ros.h>
// #include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/string.h>
#include <string.h>

#define ESR_MAX_TRACK_NUMBER 64
#define ESR_MAX_TRACK_EXTRAS_NUMBER 10
#define ESR_TRACK_START 0x500
#define ESR_TRACK_END 0x53F
#define ESR_TRACK_EXTRAS 0x540
#define ESR_VEHICLE_INFO_1 0x4E0

#define DEG_TO_RAD 0.017453293

namespace radar_interface {
class CANInterfaceESR {
public:
  CANInterfaceESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                  can::DriverInterfaceSharedPtr driver, std::string radar_name);

private:
  void frameCallback(const can::Frame &f);
  void stateCallback(const can::State &s);
  void aggregateTracks(const can::Frame &f);
  void parseTrack(const can::Frame &f);
  void parseTrackExtras(const can::Frame &f);
  void parseVehicleInfo1(const can::Frame &f);
  ros::Publisher can_topic_;
  ros::Publisher track_array_topic_;
  ros::Publisher vehicle_info_topic_;
  can::DriverInterfaceSharedPtr driver_;

  // can::CommInterface::FrameListener::Ptr frame_listener_;
  // can::StateInterface::StateListener::Ptr state_listener_;
  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  radar_interface::RadarTrackArray tracks_msg_;
  radar_interface::VehicleInfo vehicle_info_msg_;
  bool first_track_arrived_;
  bool last_track_arrived_;
  int track_count_;
  int track_extras_count_;
  std::string radar_name_;
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

can_tools::CANParseInfo ESR_TRACK_LAT_RATE(7,    // start_bit
                                           6,    // length
                                           true, // is_signed
                                           0.25, // scale
                                           0.0,  // offset
                                           -8,   // min
                                           7.75  // max
);
can_tools::CANParseInfo ESR_TRACK_GROUPING(1,     // start_bit
                                           1,     // length
                                           false, // is_signed
                                           1.0,   // scale
                                           0.0,   // offset
                                           0,     // min
                                           1      // max
);
can_tools::CANParseInfo ESR_TRACK_ONCOMING(0,     // start_bit
                                           1,     // length
                                           false, // is_signed
                                           1.0,   // scale
                                           0.0,   // offset
                                           0,     // min
                                           1      // max
);
can_tools::CANParseInfo ESR_TRACK_STATUS(15,    // start_bit
                                         3,     // length
                                         false, // is_signed
                                         1.0,   // scale
                                         0.0,   // offset
                                         0,     // min
                                         7      // max
);
can_tools::CANParseInfo ESR_TRACK_ANGLE(12,    // start_bit
                                        10,    // length
                                        true,  // is_signed
                                        0.1,   // scale
                                        0.0,   // offset
                                        -51.2, // min
                                        51.1   // max
);
can_tools::CANParseInfo ESR_TRACK_RANGE(18,    // start_bit
                                        11,    // length
                                        false, // is_signed
                                        0.1,   // scale
                                        0.0,   // offset
                                        0,     // min
                                        204.7  // max
);
can_tools::CANParseInfo ESR_TRACK_BRIDGE(39,    // start_bit
                                         1,     // length
                                         false, // is_signed
                                         1,     // scale
                                         0.0,   // offset
                                         0,     // min
                                         1      // max
);
can_tools::CANParseInfo ESR_TRACK_ROLLING(38,    // start_bit
                                          1,     // length
                                          false, // is_signed
                                          1,     // scale
                                          0.0,   // offset
                                          0,     // min
                                          1      // max
);
can_tools::CANParseInfo ESR_TRACK_WIDTH(37,    // start_bit
                                        4,     // length
                                        false, // is_signed
                                        0.5,   // scale
                                        0.0,   // offset
                                        0,     // min
                                        7.5    // max
);
can_tools::CANParseInfo ESR_TRACK_RANGE_ACCEL(33,    // start_bit
                                              10,    // length
                                              true,  // is_signed
                                              0.05,  // scale
                                              0.0,   // offset
                                              -25.6, // min
                                              25.55  // max
);
can_tools::CANParseInfo ESR_TRACK_MED_RANGE_MODE(55,    // start_bit
                                                 2,     // length
                                                 false, // is_signed
                                                 1,     // scale
                                                 0.0,   // offset
                                                 0,     // min
                                                 3      // max
);
can_tools::CANParseInfo ESR_TRACK_RANGE_RATE(53,     // start_bit
                                             14,     // length
                                             true,   // is_signed
                                             0.01,   // scale
                                             0.0,    // offset
                                             -81.92, // min
                                             81.91   // max
);

can_tools::CANParseInfo ESR_VEHICLE_SPEED(50,      // start_bit
                                          11,      // length
                                          false,   // is_signed
                                          0.0625,  // scale
                                          0.0,     // offset
                                          0,       // min
                                          127.9375 // max
);

can_tools::CANParseInfo ESR_VEHICLE_YAW_RATE(47,      // start_bit
                                             12,      // length
                                             true,    // is_signed
                                             0.0625,  // scale
                                             0.0,     // offset
                                             -128,    // min
                                             127.9375 // max
);

can_tools::CANParseInfo ESR_TRACK_EXTRAS_GROUP(3,     // start_bit
                                               4,     // length
                                               false, // is_signed
                                               1.0,   // scale
                                               0.0,   // offset
                                               0,     // min
                                               9      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_1(12,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_2(20,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_3(28,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_4(36,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_5(44,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_6(52,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_AMPLITUDE_7(60,    // start_bit
                                                     5,     // length
                                                     false, // is_signed
                                                     1.0,   // scale
                                                     -10.0, // offset
                                                     -10.0, // min
                                                     21.0   // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_1(13,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_2(21,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_3(29,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_4(37,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_5(45,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_6(53,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVING_7(61,    // start_bit
                                                  1,     // length
                                                  false, // is_signed
                                                  1.0,   // scale
                                                  0,     // offset
                                                  0,     // min
                                                  1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_1(14,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_2(22,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_3(30,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_4(38,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_5(46,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_6(54,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_SLOW_7(62,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_1(15,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_2(23,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_3(31,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_4(39,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_5(47,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_6(55,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);
can_tools::CANParseInfo ESR_TRACK_EXTRAS_MOVABLE_FAST_7(63,    // start_bit
                                                        1,     // length
                                                        false, // is_signed
                                                        1.0,   // scale
                                                        0,     // offset
                                                        0,     // min
                                                        1      // max
);

}; // namespace radar_interface

#endif // ESR_CAN_INTERFACE_H
