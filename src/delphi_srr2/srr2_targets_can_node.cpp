#include "radar_interface/delphi_srr2/srr2_targets_can.h"
#include <ros/ros.h>
#include <socketcan_interface/string.h>
#include <socketcan_interface/string.h>
#include <socketcan_interface/threading.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "esr_can");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device, radar_name, left_right_both;
  nh_param.param<std::string>("can_device", can_device, "can0");
  nh_param.param<std::string>("radar_name", radar_name, "srr2");
  nh_param.param<std::string>("left_right_both", left_right_both, "both");

can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface> ();

  if (!driver->init(can_device,
                    0)) // initialize device at can_device, 0 for no loopback.
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  } else {
    ROS_INFO("Successfully connected to %s.", can_device.c_str());
  }
  
  // if (left_right_both == "left") {
  //   radar_interface::CANInterfaceSRR2 srr2_can_interface(&nh, &nh_param, driver,
  //                                                        radar_name, SRR2_LEFT);
  //   ROS_INFO("Initialized driver: LEFT SRR2.");
  // } else if (left_right_both == "right") {
  //   radar_interface::CANInterfaceSRR2 srr2_can_interface(
  //       &nh, &nh_param, driver, radar_name, SRR2_RIGHT);
  //   ROS_INFO("Initialized driver: RIGHT SRR2.");
  // } else {
    // radar_interface::CANInterfaceSRR2 srr2_can_interface(&nh, &nh_param, driver,
    //                                                      radar_name, SRR2_BOTH);
    
  
  int left_right_both_int;
  if (left_right_both == "left") {
    left_right_both_int = SRR2_LEFT;
    ROS_INFO("Initialized driver: LEFT SRR2.");
  } else if (left_right_both == "right") {
    left_right_both_int = SRR2_RIGHT;
    ROS_INFO("Initialized driver: RIGHT SRR2.");
  } else {
    left_right_both_int = SRR2_BOTH;
      ROS_INFO("Initialized driver: LEFT and RIGHT SRR2.");
  }
  radar_interface::CANInterfaceSRR2 srr2_can_interface(&nh, &nh_param, driver,
                                                         radar_name, left_right_both_int);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}