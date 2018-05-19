#ifndef ESR_CAN_INTERFACE_H
#define ESR_CAN_INTERFACE_H

#define ESR_MAX_TRACK_NUMBER 64
#define ESR_TRACK_START 0x500
#define ESR_TRACK_END 0x53F

// Message parse information

can_tools::CANParseValueInfo ESR_ = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 0,
    .END_BYTE = 1,
    .SHIFT = 5,
    .SCALE = 0.0625,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 127.9375,
    .MASK = {0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/can_tools.h"
#include "radar_interface/delphi_esr/esr_tracks_can.h"
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/socketcan.h>
#include <string.h>

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
  int track_count_ = 0;
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

}; // namespace radar_interface

#endif // ESR_CAN_INTERFACE_H