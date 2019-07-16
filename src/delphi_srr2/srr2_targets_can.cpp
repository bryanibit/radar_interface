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

#include "radar_interface/delphi_srr2/srr2_targets_can.h"
#include <can_msgs/Frame.h>
#include <socketcan_interface/string.h>
#include <string>

namespace radar_interface {
CANInterfaceSRR2::CANInterfaceSRR2(ros::NodeHandle *nh,
                                   ros::NodeHandle *nh_param,
                                   can::DriverInterfaceSharedPtr driver,
                                   std::string radar_name,
                                   int left_right_both) {

  use_left_ = left_right_both == SRR2_LEFT || left_right_both == SRR2_BOTH;
  use_right_ = left_right_both == SRR2_RIGHT || left_right_both == SRR2_BOTH;
  can_topic_ = nh->advertise<can_msgs::Frame>("can_raw", 10);
  radar_name_ = radar_name;
  std::cout << use_left_ << "," << use_right_ << std::endl;
  if (use_left_) {
    target_array_topic_left_ =
        nh->advertise<radar_interface::RadarTargetArray>("targets_left", 10);
    radar_name_left_ = radar_name_ + "_left";
    targets_msg_left_.targets.resize(SRR2_MAX_TARGET_NUMBER);
    targets_msg_left_.header.frame_id = radar_name_left_;

    first_target_arrived_left_ = false;
    last_target_arrived_left_ = false;
    target_count_left_ = 0;
    std::cout << "left" << std::endl;
  }

  if (use_right_) {
    target_array_topic_right_ =
        nh->advertise<radar_interface::RadarTargetArray>("targets_right", 10);
    radar_name_right_ = radar_name_ + "_right";
    targets_msg_right_.targets.resize(SRR2_MAX_TARGET_NUMBER);
    targets_msg_right_.header.frame_id = radar_name_right_;

    first_target_arrived_right_ = false;
    last_target_arrived_right_ = false;
    target_count_right_ = 0;
    std::cout << "right" << std::endl;
  }
  driver_ = driver;

  // timestamp management
  nh_param->param<bool>("use_dsp_timestamps", use_dsp_timestamps_, bool(false));
  std::cout << "Uses DSP timestamps: " << use_dsp_timestamps_ << ".\n";
  first_timestamp_received_left_ = false;
  first_timestamp_received_right_ = false;

  // register handler for frames and state changes.
  frame_listener_ =
      driver_->createMsgListener(can::CommInterface::FrameDelegate(
          this, &CANInterfaceSRR2::frameCallback));

  state_listener_ =
      driver_->createStateListener(can::StateInterface::StateDelegate(
          this, &CANInterfaceSRR2::stateCallback));
};

void CANInterfaceSRR2::frameCallback(const can::Frame &f) {
  // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
  can::Frame frame = f; // copy the frame first, cannot call isValid() on const.

  // std::cout << "callback" << std::endl;
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

  if (use_dsp_timestamps_ && use_left_ && f.id == SRR2_LEFT_STATUS1 &&
      first_timestamp_received_left_) {

    double old_timestamp = timestamp_left_;
    SRR2_TIMESTAMP.parseValue(f, &timestamp_left_);
    double timestamp_delta = timestamp_left_ - old_timestamp;
    if (timestamp_delta < 0.0) {
      timestamp_delta += 65.536;
    }
    absolute_timestamp_left_ += timestamp_delta;
    std::cout << "1    " << timestamp_delta << "\n";
  } else if (use_dsp_timestamps_ && use_left_ && f.id == SRR2_LEFT_STATUS1 &&
             ~first_timestamp_received_left_) {
    first_timestamp_received_left_ = true;

    SRR2_TIMESTAMP.parseValue(f, &timestamp_left_);
    absolute_timestamp_left_ = 0.0;

    std::cout << "2    " << timestamp_left_ << "\n";
  }
  if (use_dsp_timestamps_ && use_right_ && f.id == SRR2_RIGHT_STATUS1 &&
      first_timestamp_received_right_) {
    double old_timestamp = timestamp_right_;
    SRR2_TIMESTAMP.parseValue(f, &timestamp_right_);
    double timestamp_delta = timestamp_right_ - old_timestamp;
    if (timestamp_delta < 0.0) {
      timestamp_delta += 65.536;
    }
    absolute_timestamp_right_ += timestamp_delta;
  } else if (use_dsp_timestamps_ && use_right_ && f.id == SRR2_RIGHT_STATUS1 &&
             ~first_timestamp_received_right_) {
    SRR2_TIMESTAMP.parseValue(f, &timestamp_right_);
    absolute_timestamp_right_ = 0.0;
    first_timestamp_received_right_ = true;
  }

  if (use_left_ && f.id >= SRR2_LEFT_TARGET_START &&
      f.id <= SRR2_LEFT_TARGET_END) {
    parseTarget(f, true);
    aggregateTargets(f, true);
  }
  if (use_right_ && f.id >= SRR2_RIGHT_TARGET_START &&
      f.id <= SRR2_RIGHT_TARGET_END) {
    parseTarget(f, false);
    aggregateTargets(f, false);
  }

  can_msgs::Frame raw_can_msg;
  // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
  convertSocketCANToMessage(f, raw_can_msg);
  // std::cout << "Publish" << std::endl;
  raw_can_msg.header.frame_id = radar_name_;
  raw_can_msg.header.stamp = ros::Time::now();

  can_topic_.publish(raw_can_msg);
};

void CANInterfaceSRR2::parseTarget(const can::Frame &f, bool is_left) {
  uint8_t valid_level, target_id;
  float range, range_rate, azimuth, rcs;
  bool status;

  if (is_left) {
    target_id = f.id - SRR2_LEFT_TARGET_START;
  } else {
    target_id = f.id - SRR2_RIGHT_TARGET_START;
  }
  SRR2_TARGET_RANGE.parseValue(f, &range);
  SRR2_TARGET_RANGE_RATE.parseValue(f, &range_rate);
  SRR2_TARGET_ANGLE.parseValue(f, &azimuth);
  SRR2_TARGET_AMPLITUDE.parseValue(f, &rcs);

  SRR2_TARGET_STATUS.parseValue(f, &status);
  SRR2_TARGET_VALID_LEVEL.parseValue(f, &valid_level);
  azimuth = azimuth * DEG_TO_RAD;

  if (is_left) {
    targets_msg_left_.targets[target_id].range = range;
    targets_msg_left_.targets[target_id].azimuth = azimuth;
    targets_msg_left_.targets[target_id].range_rate = range_rate;
    targets_msg_left_.targets[target_id].rcs = rcs;
    if (status) {
      targets_msg_left_.targets[target_id].status = valid_level;
    } else {
      targets_msg_left_.targets[target_id].status = -1;
    }
  } else {
    targets_msg_right_.targets[target_id].range = range;
    targets_msg_right_.targets[target_id].azimuth = azimuth;
    targets_msg_right_.targets[target_id].range_rate = range_rate;
    targets_msg_right_.targets[target_id].rcs = rcs;
    if (status) {
      targets_msg_right_.targets[target_id].status = valid_level;
    } else {
      targets_msg_right_.targets[target_id].status = -1;
    }
  }
};

void CANInterfaceSRR2::aggregateTargets(const can::Frame &f, bool is_left) {
  int target_id;
  bool to_publish = false;
  if (is_left) {
    target_id = f.id - SRR2_LEFT_TARGET_START;
    if (target_id == 0) {
      if (use_dsp_timestamps_) {
        targets_msg_left_.header.stamp = ros::Time(absolute_timestamp_left_);
      } else {
        targets_msg_left_.header.stamp = ros::Time::now();
      }
      first_target_arrived_left_ = true;
      target_count_left_ = 1;
    } else {
      target_count_left_ += 1;
    }
    if (target_count_left_ == SRR2_MAX_TARGET_NUMBER) {
      to_publish = true;
      target_array_topic_left_.publish(targets_msg_left_);
    }
  } else {
    target_id = f.id - SRR2_RIGHT_TARGET_START;
    if (target_id == 0) {
      if (use_dsp_timestamps_) {
        targets_msg_right_.header.stamp = ros::Time(absolute_timestamp_right_);
      } else {
        targets_msg_right_.header.stamp = ros::Time::now();
      }
      first_target_arrived_right_ = true;
      target_count_right_ = 1;
    } else {
      target_count_right_ += 1;
    }
    if (target_count_right_ == SRR2_MAX_TARGET_NUMBER) {
      to_publish = true;
      target_array_topic_right_.publish(targets_msg_right_);
    }
  }
};

void CANInterfaceSRR2::stateCallback(const can::State &s) {
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