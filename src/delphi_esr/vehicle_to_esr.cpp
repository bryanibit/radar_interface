#include "radar_interface/delphi_esr/vehicle_to_esr.h"

VehicleToESR::VehicleToESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                           boost::shared_ptr<can::DriverInterface> driver)
    : socketcan_bridge::TopicToSocketCAN(nh, nh_param, driver) {
  twist_topic_ = nh->subscribe<geometry_msgs::Twist>(
      "/vehicle_twist", 10,
      boost::bind(&VehicleToESR::twistCallback, this, _1));
  driver_interface_ = driver;

  setFrameProperties(&frame_vehicle1_);
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

void VehicleToESR::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  twist_ = *msg.get();
}

void VehicleToESR::sendCanFrame(const ros::TimerEvent &event) {

  bool always_true = true;
  frame_vehicle1_.id = VEH_VEL.MSG_ID;
  frame_vehicle1_.id = VEH_YAW_RATE.MSG_ID;
  float speed_abs = std::abs(twist_.linear.x);
  bool speed_sign = std::signbit(twist_.linear.x);
  float yaw_rate_deg = twist_.angular.z * RAD_TO_DEG;

  can_tools::setValue(&frame_vehicle1_, speed_abs, VEH_VEL);
  can_tools::setValue(&frame_vehicle1_, speed_sign, VEH_VEL_DIR);
  can_tools::setValue(&frame_vehicle1_, yaw_rate_deg, VEH_YAW_RATE);
  can_tools::setValue(&frame_vehicle1_, always_true, VEH_YAW_RATE_VALID);

  bool res = driver_interface_->send(frame_vehicle1_);

  if (!res) {
    ROS_ERROR("Failed to send message: %s.",
              can::tostring(frame_vehicle1_, true).c_str());
  } else {
    ROS_INFO("Sent message: %s.", can::tostring(frame_vehicle1_, true).c_str());
  }
  // res = driver_interface_->send(frame_yaw_rate);
  // if (!res) {
  //   ROS_ERROR("Failed to send message: %s.",
  //             can::tostring(frame_yaw_rate, true).c_str());
  // }
}