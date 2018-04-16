#include "geometry_msgs/Twist.h"
#include "radar_interface/can_tools.h"
#include "socketcan_bridge/topic_to_socketcan.h"
#include <can_msgs/Frame.h>
#include <ros/ros.h>

class VehicleToSRR2 : public socketcan_bridge::TopicToSocketCAN {
public:
  VehicleToSRR2(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
                boost::shared_ptr<can::DriverInterface> driver,
                std::string topic_name);
  virtual ~VehicleToSRR2();

private:
  ros::Subscriber twist_topic_;
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
};