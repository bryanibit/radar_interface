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

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class SubsriberPublisher {
public:
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::NodeHandle n;
  void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);

private:
  /* data */
};
void SubsriberPublisher::imuCallback(
    const sensor_msgs::Imu::ConstPtr &imu_msg) {
  geometry_msgs::Twist twist;
  twist.angular.z = imu_msg->angular_velocity.z;
  pub.publish(twist);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "imu_to_vehicle_node");

  SubsriberPublisher sp;
  sp.sub =
      sp.n.subscribe("/imu/data", 1000, &SubsriberPublisher::imuCallback, &sp);
  sp.pub = sp.n.advertise<geometry_msgs::Twist>("vehicle_twist", 1000);

  ros::spin();

  return 0;
}
