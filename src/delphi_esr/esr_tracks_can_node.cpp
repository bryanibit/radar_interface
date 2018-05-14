#include "radar_interface/delphi_esr/esr_tracks_can_node.h"
#include <ros/ros.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <string>



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "socketcan_to_topic_node");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device;
  nh_param.param<std::string>("can_device", can_device, "can0");

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

  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver);
  to_topic_bridge.setup();

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}