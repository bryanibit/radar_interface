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
