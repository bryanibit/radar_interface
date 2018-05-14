#include "ros/ros.h"
#include "std_msgs/String.h"



void imuCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "imu_to_vehicle_node");

	
	ros::NodeHandle n;


	ros::Subscriber sub = n.subscribe("/data/imu", 1000, imuCallback);

	ros::spin();

	return 0;
}