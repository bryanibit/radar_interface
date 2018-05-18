#ifndef ESR_CAN_INTERFACE_H
#define ESR_CAN_INTERFACE_H

#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/can_tools.h"
#include "radar_interface/delphi_esr/esr_tracks_can.h"
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/socketcan.h>
#include <string.h>

#define ESR_MAX_TRACK_NUMBER 64
#define ESR_TRACK_START 0x500
#define ESR_TRACK_END 0x53F

namespace radar_interface {
class CANInterfaceESR {
public:
  CANInterfaceESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                  can::DriverInterfaceSharedPtr driver);

private:
  void frameCallback(const can::Frame &f);
  void stateCallback(const can::State &s);
  void setRadarName(std::string &name){radar_name_ = name};
  ros::Publisher can_topic_;
  ros::Publisher track_array_topic_;
  can::DriverInterfaceSharedPtr driver_;

  can::FrameListenerConstSharedPtr frame_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  RadarTrackArray tracks_msg_;
  bool first_track_arrived = false;
  bool last_track_arrived = false;
  int track_count = 0;
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
can_tools::CANParseValueInfo ESR_TRACK_LAT_RATE = {
    .MSG_ID = 0x500,
    .START_BYTE = 0,
    .END_BYTE = 0,
    .SHIFT = 2,
    .SCALE = 0.25,
    .OFFSET = 0,
    .MIN = -8,
    .MAX = 7.75,
    .MASK = {0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_GROUPING = {
    .MSG_ID = 0x500,
    .START_BYTE = 0,
    .END_BYTE = 0,
    .SHIFT = 1,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_ONCOMING = {
    .MSG_ID = 0x500,
    .START_BYTE = 0,
    .END_BYTE = 0,
    .SHIFT = 0,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_STATUS = {
    .MSG_ID = 0x500,
    .START_BYTE = 1,
    .END_BYTE = 1,
    .SHIFT = 5,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 7,
    .MASK = {0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_ANGLE = {
    .MSG_ID = 0x500,
    .START_BYTE = 1,
    .END_BYTE = 2,
    .SHIFT = 3,
    .SCALE = 0.1,
    .OFFSET = 0,
    .MIN = -51.2,
    .MAX = 51.1,
    .MASK = {0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_RANGE = {
    .MSG_ID = 0x500,
    .START_BYTE = 2,
    .END_BYTE = 3,
    .SHIFT = 0,
    .SCALE = 0.1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 204.7,
    .MASK = {0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_BRIDGE = {
    .MSG_ID = 0x500,
    .START_BYTE = 4,
    .END_BYTE = 4,
    .SHIFT = 7,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_ROLLING = {
    .MSG_ID = 0x500,
    .START_BYTE = 4,
    .END_BYTE = 4,
    .SHIFT = 6,
    .SCALE = 0.1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_WIDTH = {
    .MSG_ID = 0x500,
    .START_BYTE = 4,
    .END_BYTE = 4,
    .SHIFT = 2,
    .SCALE = 0.5,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 7.5,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_RANGE_ACCEL = {
    .MSG_ID = 0x500,
    .START_BYTE = 4,
    .END_BYTE = 5,
    .SHIFT = 0,
    .SCALE = 0.05,
    .OFFSET = 0,
    .MIN = -25.6,
    .MAX = 25.55,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x00, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_MED_RANGE_MODE = {
    .MSG_ID = 0x500,
    .START_BYTE = 6,
    .END_BYTE = 6,
    .SHIFT = 6,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 3,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00}};
can_tools::CANParseValueInfo ESR_TRACK_RANGE_RATE = {
    .MSG_ID = 0x500,
    .START_BYTE = 6,
    .END_BYTE = 7,
    .SHIFT = 0,
    .SCALE = 0.1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 320,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff}};

}; // namespace radar_interface

#endif // ESR_CAN_INTERFACE_H