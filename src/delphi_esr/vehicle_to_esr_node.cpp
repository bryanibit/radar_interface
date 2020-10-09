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
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <socketcan_interface/string.h>
#include <socketcan_interface/threading.h>
#include <string>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "socketcan_to_topic_node");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device;
  nh_param.param<std::string>("can_device", can_device, "can0");

  // boost::shared_ptr<can::ThreadedSocketCANInterface> driver =
  // boost::make_shared<can::ThreadedSocketCANInterface> ();
  can::ThreadedSocketCANInterfaceSharedPtr driver =
      boost::make_shared<can::ThreadedSocketCANInterface>();

  if (!driver->init(can_device,
                    0)) // initialize device at can_device, 0 for no loopback.
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  } else {
    ROS_INFO("Successfully connected to %s.", can_device.c_str());
  }

  VehicleToESR to_topic_bridge(&nh, &nh_param, driver);
  to_topic_bridge.setup();

  ros::Timer timer = nh.createTimer(
      ros::Duration(0.1), &VehicleToESR::sendCanFrame, &to_topic_bridge);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}
