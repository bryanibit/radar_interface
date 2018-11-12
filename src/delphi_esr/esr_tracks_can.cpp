#include "radar_interface/delphi_esr/esr_tracks_can.h"
#include <can_msgs/Frame.h>
#include <socketcan_interface/string.h>
#include <string>

namespace radar_interface {
CANInterfaceESR::CANInterfaceESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                                 can::DriverInterfaceSharedPtr driver,
                                 std::string radar_name) {
  can_topic_ = nh->advertise<can_msgs::Frame>("can_raw", 10);
  track_array_topic_ =
      nh->advertise<radar_interface::RadarTrackArray>("tracks", 10);
  driver_ = driver;
  radar_name_ = radar_name;

  // register handler for frames and state changes.
  frame_listener_ = driver_->createMsgListener(
      can::CommInterface::FrameDelegate(this, &CANInterfaceESR::frameCallback));

  state_listener_ =
      driver_->createStateListener(can::StateInterface::StateDelegate(
          this, &CANInterfaceESR::stateCallback));

  tracks_msg_.tracks.resize(ESR_MAX_TRACK_NUMBER);
  tracks_msg_.header.frame_id = radar_name_;

  first_track_arrived_ = false;
  last_track_arrived_ = false;
  track_count_ = 0;
};

void CANInterfaceESR::frameCallback(const can::Frame &f) {
  // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
  can::Frame frame = f; // copy the frame first, cannot call isValid() on const.
  if (!frame.isValid()) {
    ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, "
              "is_extended: %d, is_error: %d, is_rtr: %d",
              f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
    return;
  } else {
    if (f.is_error) {
      // can::tostring cannot be used for dlc > 8 frames. It causes an crash
      // due to usage of boost::array for the data array. The should always
      // work.
      ROS_WARN(
          "Received frame is error"); //: %s", can::tostring(f, true).c_str());
    }
  }

  if (f.id >= ESR_TRACK_START && f.id <= ESR_TRACK_END) {
    // std::cout << f.id;
    parseTrack(f);

    aggregateTracks(f);
  }
  can_msgs::Frame raw_can_msg;
  // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
  convertSocketCANToMessage(f, raw_can_msg);

  raw_can_msg.header.frame_id = radar_name_;
  raw_can_msg.header.stamp = ros::Time::now();

  can_topic_.publish(raw_can_msg);
};

void CANInterfaceESR::parseTrack(const can::Frame &f) {
  int track_id, status;
  float range, range_rate, range_accel, azimuth, lat_rate, width;

  track_id = f.id - ESR_TRACK_START;
  ESR_TRACK_RANGE.parseValue(f, &range);
  ESR_TRACK_RANGE_RATE.parseValue(f, &range_rate);
  ESR_TRACK_RANGE_ACCEL.parseValue(f, &range_accel);
  ESR_TRACK_ANGLE.parseValue(f, &azimuth);
  // std::cout << azimuth << std::endl;

  ESR_TRACK_LAT_RATE.parseValue(f, &lat_rate);
  ESR_TRACK_STATUS.parseValue(f, &status);
  ESR_TRACK_WIDTH.parseValue(f, &width);

  // if (status>0 && (abs(range_rate)>1 || abs(lat_rate)>1 ||
  // abs(range_accel)>1)){
  // ROS_INFO("%d,%d,%f,%f,%f,%f,%f", track_id,status, azimuth, range,
  // range_rate, lat_rate,
  //          range_accel);
  //          }
  azimuth = azimuth * DEG_TO_RAD;
  tracks_msg_.tracks[track_id].id = track_id;
  tracks_msg_.tracks[track_id].pos.x = cos(azimuth) * range;
  tracks_msg_.tracks[track_id].pos.y = sin(azimuth) * range;
  tracks_msg_.tracks[track_id].vel.x =
      cos(azimuth) * range_rate + sin(azimuth) * lat_rate;
  tracks_msg_.tracks[track_id].vel.y =
      sin(azimuth) * range_rate + cos(azimuth) * lat_rate;
  tracks_msg_.tracks[track_id].acc.x = cos(azimuth) * range_accel;
  tracks_msg_.tracks[track_id].acc.y = sin(azimuth) * range_accel;

  tracks_msg_.tracks[track_id].status = status;
  tracks_msg_.tracks[track_id].width = width;
  if (width > 0.0) {
    std::cout << width << std::endl;
  }
};

void CANInterfaceESR::aggregateTracks(const can::Frame &f) {
  int track_id;
  bool to_publish = false;
  track_id = f.id - ESR_TRACK_START;

  if (track_id == 0) {
    tracks_msg_.header.stamp = ros::Time::now();
    first_track_arrived_ = true;
    track_count_ = 1;
  } else {
    track_count_ += 1;
  }
  if (track_count_ == ESR_MAX_TRACK_NUMBER) {
    to_publish = true;
    track_array_topic_.publish(tracks_msg_);
  }
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