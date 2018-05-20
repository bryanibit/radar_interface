#include "radar_interface/delphi_esr/esr_tracks_can.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "esr_can");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device, radar_name;
  nh_param.param<std::string>("can_device", can_device, "can0");
  nh_param.param<std::string>("radar_name", radar_name, "esr");

  boost::shared_ptr<can::ThreadedSocketCANInterface> driver =
      boost::make_shared<can::ThreadedSocketCANInterface>();

  if (!driver->init(can_device,
                    0)) // initialize device at can_device, 0 for no loopback.
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  } else {
    ROS_INFO("Successfully connected to %s.", can_device.c_str());
  }

  radar_interface::CANInterfaceESR esr_can_interface(&nh, &nh_param, driver,
                                                     radar_name);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}