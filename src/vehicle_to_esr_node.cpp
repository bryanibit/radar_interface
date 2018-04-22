#include "ros/ros.h"
#include "std_msgs/String.h"
#include "radar_interface/vehicle_to_esr.h"
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <string>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "socketcan_to_topic_node");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device;
  nh_param.param<std::string>("can_device", can_device, "vcan0");

  boost::shared_ptr<can::ThreadedSocketCANInterface> driver = boost::make_shared<can::ThreadedSocketCANInterface> ();

  if (!driver->init(can_device, 0))  // initialize device at can_device, 0 for no loopback.
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  }
    else
  {
    ROS_INFO("Successfully connected to %s.", can_device.c_str());
  }

  VehicleToESR to_topic_bridge(&nh, &nh_param, driver);
  to_topic_bridge.setup();

  ros::Timer timer = nh.createTimer(ros::Duration(1), &VehicleToESR::sendCanFrame, &to_topic_bridge);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}