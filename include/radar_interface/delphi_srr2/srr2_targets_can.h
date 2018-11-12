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
#define SRR2_RIGHT_TARGET_START 0x600
#define SRR2_RIGHT_TARGET_END 0x63F
#define SRR2_LEFT 1
#define SRR2_RIGHT 2
#define SRR2_BOTH 3
#define DEG_TO_RAD 0.017453293

namespace radar_interface {
class CANInterfaceSRR2 {
public:
  CANInterfaceSRR2(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                   boost::shared_ptr<can::DriverInterface> driver,
                   std::string radar_name, int left_right_both);

private:
  void frameCallback(const can::Frame &f);
  void stateCallback(const can::State &s);
  void aggregateTargets(const can::Frame &f, bool is_left);
  void parseTarget(const can::Frame &f, bool is_left);
  ros::Publisher can_topic_;
  ros::Publisher target_array_topic_left_;
  ros::Publisher target_array_topic_right_;
  boost::shared_ptr<can::DriverInterface> driver_;

  // can::CommInterface::FrameListener::Ptr frame_listener_;
  // can::StateInterface::StateListener::Ptr state_listener_;
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
can_tools::CANParseValueInfo SRR2_TARGET_VALID_LEVEL = {
    .MSG_ID = 0x500,
    .START_BYTE = 0,
    .END_BYTE = 0,
    .SHIFT = 5,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 7,
    .MASK = {0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo SRR2_TARGET_STATUS = {
    .MSG_ID = 0x500,
    .START_BYTE = 0,
    .END_BYTE = 0,
    .SHIFT = 4,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo SRR2_TARGET_AMPLITUDE = {
    .MSG_ID = 0x500,
    .START_BYTE = 0,
    .END_BYTE = 1,
    .SHIFT = 0,
    .SCALE = 0.125,
    .OFFSET = 0,
    .MIN = -24,
    .MAX = 40,
    .MASK = {0x0f, 0xff, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo SRR2_TARGET_ANGLE = {
    .MSG_ID = 0x500,
    .START_BYTE = 2,
    .END_BYTE = 3,
    .SHIFT = 0,
    .SCALE = 0.0078125,
    .OFFSET = 0,
    .MIN = -102.4,
    .MAX = 102.2,
    .MASK = {0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo SRR2_TARGET_RANGE = {
    .MSG_ID = 0x500,
    .START_BYTE = 4,
    .END_BYTE = 5,
    .SHIFT = 0,
    .SCALE = 0.0078125,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 204.7,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}};
can_tools::CANParseValueInfo SRR2_TARGET_RANGE_RATE = {
    .MSG_ID = 0x500,
    .START_BYTE = 6,
    .END_BYTE = 7,
    .SHIFT = 0,
    .SCALE = 0.0078125,
    .OFFSET = 0,
    .MIN = -81.92,
    .MAX = 81.91,
    .MASK = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff}};

}; // namespace radar_interface

#endif // SRR2_CAN_INTERFACE_H
