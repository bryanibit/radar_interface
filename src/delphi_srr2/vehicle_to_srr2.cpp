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

#include "radar_interface/delphi_srr2/vehicle_to_srr2.h"

VehicleToSRR2::VehicleToSRR2(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                             can::DriverInterfaceSharedPtr driver)
    : socketcan_bridge::TopicToSocketCAN(nh, nh_param, driver) {
  twist_topic_ = nh->subscribe<geometry_msgs::Twist>(
      "/vehicle_twist", 10,
      boost::bind(&VehicleToSRR2::twistCallback, this, _1));
  driver_interface_ = driver;

  setFrameProperties(&frame_vel);
  setFrameProperties(&frame_yaw_rate);
}

VehicleToSRR2::~VehicleToSRR2() {}

void VehicleToSRR2::setFrameProperties(can::Frame *frame) {
  frame->is_extended = false;
  frame->is_rtr = false;
  frame->is_error = false;
  frame->dlc = 8;
  for (size_t i = 0; i < 8; i++) {
    frame->data[i] = 0;
  }
}

void VehicleToSRR2::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  twist_ = *msg.get();
}

void VehicleToSRR2::sendCanFrame(const ros::TimerEvent &event) {

  bool always_true = true;
  uint8_t speed_qf = 3;
  frame_vel.id = VEH_VEL.MSG_ID;
  frame_yaw_rate.id = VEH_YAW_RATE.MSG_ID;
  float speed_abs = std::abs(twist_.linear.x);

  can_tools::setValue(&frame_vel, speed_abs, VEH_VEL);
  can_tools::setValue(&frame_vel, always_true, VEH_VEL_UB);
  can_tools::setValue(&frame_vel, speed_qf, VEH_VEL_QF);
  can_tools::setValue(&frame_yaw_rate, twist_.angular.z, VEH_YAW_RATE);

  bool res = driver_interface_->send(frame_vel);
  if (!res) {
    ROS_ERROR("Failed to send message: %s.",
              can::tostring(frame_vel, true).c_str());
  }
  // res = driver_interface_->send(frame_yaw_rate);
  // if (!res) {
  //   ROS_ERROR("Failed to send message: %s.",
  //             can::tostring(frame_yaw_rate, true).c_str());
  // }
}