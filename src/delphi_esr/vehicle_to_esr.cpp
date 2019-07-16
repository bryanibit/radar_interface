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

#include "radar_interface/delphi_esr/vehicle_to_esr.h"

VehicleToESR::VehicleToESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                           can::DriverInterfaceSharedPtr driver)
    : socketcan_bridge::TopicToSocketCAN(nh, nh_param, driver) {
  twist_topic_ = nh->subscribe<geometry_msgs::TwistStamped>(
      "vehicle_twist", 10, boost::bind(&VehicleToESR::twistCallback, this, _1));
  driver_interface_ = driver;

  setFrameProperties(&frame_vehicle1_);
  setFrameProperties(&frame_vehicle2_);
}

VehicleToESR::~VehicleToESR() {}

void VehicleToESR::setFrameProperties(can::Frame *frame) {
  frame->is_extended = false;
  frame->is_rtr = false;
  frame->is_error = false;
  frame->dlc = 8;
  for (size_t i = 0; i < 8; i++) {
    frame->data[i] = 0;
  }
}

void VehicleToESR::twistCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
  twist_ = *msg.get();
  ROS_INFO("Received speed: %f.", twist_.twist.linear.x);
}

void VehicleToESR::sendCanFrame(const ros::TimerEvent &event) {

  bool always_true = true;
  int number_of_tracks = ESR_MAX_TRACKS;
  int grouping_mode = ESR_GROUPING_MODE;

  frame_vehicle1_.id = ESR_VEHICLE_INFO_MSG_ID_TX_1;
  frame_vehicle2_.id = ESR_VEHICLE_INFO_MSG_ID_TX_2;

  float speed_abs = std::abs(twist_.twist.linear.x);
  bool speed_sign = std::signbit(twist_.twist.linear.x);
  float yaw_rate_deg = -twist_.twist.angular.z * RAD_TO_DEG;

  // can_tools::setValue(&frame_vehicle1_, speed_abs, VEH_VEL);
  // can_tools::setValue(&frame_vehicle1_, speed_sign, VEH_VEL_DIR);
  // can_tools::setValue(&frame_vehicle1_, yaw_rate_deg, VEH_YAW_RATE);
  // can_tools::setValue(&frame_vehicle1_, always_true, VEH_YAW_RATE_VALID);

  VEH_YAW_RATE_VALID.setValue(&frame_vehicle1_, always_true);
  VEH_YAW_RATE.setValue(&frame_vehicle1_, yaw_rate_deg);

  VEH_VEL.setValue(&frame_vehicle1_, speed_abs);
  VEH_VEL_DIR.setValue(&frame_vehicle1_, speed_sign);
  VEH_VEL_VALID.setValue(&frame_vehicle2_, always_true);
  MAX_TRACKS.setValue(&frame_vehicle2_, number_of_tracks);
  TO_RADIATE.setValue(&frame_vehicle2_, always_true);
  GROUPING_MODE.setValue(&frame_vehicle2_, grouping_mode);

  bool res = driver_interface_->send(frame_vehicle1_);

  if (!res) {
    ROS_ERROR("Failed to send message: %s.",
              can::tostring(frame_vehicle1_, true).c_str());
  } else {
    ROS_INFO("Sent message: %s.", can::tostring(frame_vehicle1_, true).c_str());
  }

  res = driver_interface_->send(frame_vehicle2_);

  if (!res) {
    ROS_ERROR("Failed to send message: %s.",
              can::tostring(frame_vehicle1_, true).c_str());
  } else {
    ROS_INFO("Sent message: %s.", can::tostring(frame_vehicle2_, true).c_str());
  }

  // res = driver_interface_->send(frame_yaw_rate);
  // if (!res) {
  //   ROS_ERROR("Failed to send message: %s.",
  //             can::tostring(frame_yaw_rate, true).c_str());
  // }
}
