#ifndef ESR_CAN_INTERFACE_H
#define ESR_CAN_INTERFACE_H

#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/can_tools.h"
#include <can_msgs/Frame.h>
#include <ros/ros.h>
// #include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/string.h>
#include <string.h>

#define ESR_MAX_TRACK_NUMBER 64
#define ESR_TRACK_START 0x500
#define ESR_TRACK_END 0x53F

#define DEG_TO_RAD 0.017453293

namespace radar_interface {
class CANInterfaceESR {
public:
  CANInterfaceESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                  can::DriverInterfaceSharedPtr driver,
                  std::string radar_name);

private:
  void frameCallback(const can::Frame &f);
  void stateCallback(const can::State &s);
  void aggregateTracks(const can::Frame &f);
  void parseTrack(const can::Frame &f);
  ros::Publisher can_topic_;
  ros::Publisher track_array_topic_;
  can::DriverInterfaceSharedPtr driver_;

  // can::CommInterface::FrameListener::Ptr frame_listener_;
  // can::StateInterface::StateListener::Ptr state_listener_;
  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  radar_interface::RadarTrackArray tracks_msg_;
  bool first_track_arrived_;
  bool last_track_arrived_;
  int track_count_;
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
                                        1,    // length
                                        false, // is_signed
                                        1,   // scale
                                        0.0,   // offset
                                        0,     // min
                                        1  // max
                                        );
can_tools::CANParseInfo ESR_TRACK_ROLLING(38,    // start_bit
                                        1,    // length
                                        false, // is_signed
                                        1,   // scale
                                        0.0,   // offset
                                        0,     // min
                                        1  // max
                                        );
can_tools::CANParseInfo ESR_TRACK_WIDTH(37,    // start_bit
                                        4,    // length
                                        false, // is_signed
                                        0.5,   // scale
                                        0.0,   // offset
                                        0,     // min
                                        7.5  // max
                                        );
can_tools::CANParseInfo ESR_TRACK_RANGE_ACCEL(33,    // start_bit
                                        10,    // length
                                        true, // is_signed
                                        0.05,   // scale
                                        0.0,   // offset
                                        -25.6,     // min
                                        25.55  // max
                                        );
can_tools::CANParseInfo ESR_TRACK_MED_RANGE_MODE(55,    // start_bit
                                        2,    // length
                                        false, // is_signed
                                        1,   // scale
                                        0.0,   // offset
                                        0,     // min
                                        3  // max
                                        );
can_tools::CANParseInfo ESR_TRACK_RANGE_RATE(53,    // start_bit
                                        14,    // length
                                        true, // is_signed
                                        0.01,   // scale
                                        0.0,   // offset
                                        -81.92,     // min
                                        81.91  // max
                                        );

// can_tools::CANParseValueInfo ESR_TRACK_LAT_RATE = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 0,
//     .END_BYTE = 0,
//     .SHIFT = 2,
//     .SCALE = 0.25,
//     .OFFSET = 0,
//     .MIN = -8,
//     .MAX = 7.75,
//     .MASK = {0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_GROUPING = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 0,
//     .END_BYTE = 0,
//     .SHIFT = 1,
//     .SCALE = 1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 1,
//     .MASK = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_ONCOMING = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 0,
//     .END_BYTE = 0,
//     .SHIFT = 0,
//     .SCALE = 1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 1,
//     .MASK = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_STATUS = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 1,
//     .END_BYTE = 1,
//     .SHIFT = 5,
//     .SCALE = 1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 7,
//     .MASK = {0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_ANGLE = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 1,
//     .END_BYTE = 2,
//     .SHIFT = 3,
//     .SCALE = 0.1,
//     .OFFSET = 0,
//     .MIN = -51.2,
//     .MAX = 51.1,
//     .MASK = {0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_RANGE = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 2,
//     .END_BYTE = 3,
//     .SHIFT = 0,
//     .SCALE = 0.1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 204.7,
//     .MASK = {0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_BRIDGE = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 4,
//     .END_BYTE = 4,
//     .SHIFT = 7,
//     .SCALE = 1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 1,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_ROLLING = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 4,
//     .END_BYTE = 4,
//     .SHIFT = 6,
//     .SCALE = 0.1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 1,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_WIDTH = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 4,
//     .END_BYTE = 4,
//     .SHIFT = 2,
//     .SCALE = 0.5,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 7.5,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_RANGE_ACCEL = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 4,
//     .END_BYTE = 5,
//     .SHIFT = 0,
//     .SCALE = 0.05,
//     .OFFSET = 0,
//     .MIN = -25.6,
//     .MAX = 25.55,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x00, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_MED_RANGE_MODE = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 6,
//     .END_BYTE = 6,
//     .SHIFT = 6,
//     .SCALE = 1,
//     .OFFSET = 0,
//     .MIN = 0,
//     .MAX = 3,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00}};
// can_tools::CANParseValueInfo ESR_TRACK_RANGE_RATE = {
//     .MSG_ID = 0x500,
//     .START_BYTE = 6,
//     .END_BYTE = 7,
//     .SHIFT = 0,
//     .SCALE = 0.01,
//     .OFFSET = 0,
//     .MIN = -81.92,
//     .MAX = 81.91,
//     .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff}};

}; // namespace radar_interface

#endif // ESR_CAN_INTERFACE_H
