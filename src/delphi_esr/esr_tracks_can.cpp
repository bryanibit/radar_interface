#include "radar_interface/delphi_esr/esr_tracks_can.h"
#include <can_msgs/Frame.h>
#include <socketcan_interface/string.h>
#include <string>

namespace radar_interface {
CANInterfaceESR::CANInterfaceESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                                 can::DriverInterfaceSharedPtr driver) {
  can_topic_ = nh->advertise<can_msgs::Frame>("can_raw", 10);
  track_array_topic_ = nh->advertise<radar_interface::RadarTrackArray>("tracks", 10);
  driver_ = driver;

  // register handler for frames and state changes.
  frame_listener_ = driver_->createMsgListener(
      can::CommInterface::FrameDelegate(this, &CANInterfaceESR::frameCallback));

  state_listener_ =
      driver_->createStateListener(can::StateInterface::StateDelegate(
          this, &CANInterfaceESR::stateCallback));

  tracks_msg_.tracks.resize(ESR_MAX_TRACK_NUMBER)
};

void CANInterfaceESR::frameCallback(const can::Frame &f) {
  // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
  if (!f.isValid()) {
    ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, "
              "is_extended: %d, is_error: %d, is_rtr: %d",
              f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
    return;
  } else {
    if (f.is_error) {
      // can::tostring cannot be used for dlc > 8 frames. It causes an crash
      // due to usage of boost::array for the data array. The should always
      // work.
      ROS_WARN("Received frame is error: %s", can::tostring(f, true).c_str());
    }
  }

  if (f.id >= ESR_TRACK_START && f.id <= ESR_TRACK_END){
      parseTrack(f);
  }
  can_msgs::Frame msg;
  // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
  convertSocketCANToMessage(f, msg);
  
  msg.header.frame_id = radar_name_; 
  msg.header.stamp = ros::Time::now();

  can_topic_.publish(msg);


};

void CANInterfaceESR::parseTrack(const can::Frame &f){
    int track_id;
    float range,range_rate,azimuth,

    track_id = f.id - ESR_TRACK_START;
};

void CANInterfaceESR::stateCallback(const can::State &s) {
  std::string err;
  driver_->translateError(s.internal_error, err);
  if (!s.internal_error) {
    ROS_INFO("State: %s, asio: %s", err.c_str(),
             s.error_code.message().c_str());
  } else {
    ROS_ERROR("Error: %s, asio: %s", err.c_str(),
              s.error_code.message().c_str());
  }
};
}; // namespace radar_interface