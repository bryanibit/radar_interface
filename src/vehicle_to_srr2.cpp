#include "radar_interface/vehicle_to_srr2.h"

VehicleToSRR2::VehicleToSRR2(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                             boost::shared_ptr<can::DriverInterface> driver,
                             std::string topic_name)
    : socketcan_bridge::TopicToSocketCAN(nh, nh_param, driver) {
  twist_topic_ = nh->subscribe<geometry_msgs::Twist>(
      topic_name, 10, boost::bind(&VehicleToSRR2::twistCallback, this, _1));
}

VehicleToSRR2::~VehicleToSRR2() {}

void VehicleToSRR2::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  geometry_msgs::Twist twist = *msg.get();
  if (twist.linear.x < 0) {}
}