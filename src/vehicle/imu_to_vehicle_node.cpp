#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "imu_to_vehicle_node");

	SubsriberPublisher sp;
  	sp.sub = sp.n.subscribe("/data/imu", 1000, sp.imuCallback);
	sp.pub = sp.n.advertise<geometry_msgs::Twist>("vehicle_twist", 1000);

	ros::spin();

	return 0;
}

class SubsriberPublisher
{
public:
	SubsriberPublisher();
	virtual ~SubsriberPublisher();
	ros::Subscriber sub;
	ros::Publisher  pub;
	ros::NodeHandle n;
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
private:
	/* data */
};

SubsriberPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
	geometry_msgs::Twist twist;
	twist.angular.z=imu.angular_velocity.z;
	pub.publish(twist);
}