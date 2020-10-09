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

#include "radar_interface/delphi_esr/esr_tracks_can.h"
#include <ros/ros.h>
#include <socketcan_interface/string.h>
#include <socketcan_interface/threading.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "esr_can");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device, radar_name;
  nh_param.param<std::string>("can_device", can_device, "can0");
  nh_param.param<std::string>("radar_name", radar_name, "esr");

  // boost::shared_ptr<can::ThreadedSocketCANInterface> driver =
  // boost::make_shared<can::ThreadedSocketCANInterface> ();
  // can::ThreadedSocketCANInterfaceSharedPtr driver;
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

  radar_interface::CANInterfaceESR esr_can_interface(&nh, &nh_param, driver,
                                                     radar_name);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}
