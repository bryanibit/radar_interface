#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "imu_to_vehicle_node");

	SubsriberPublisher sp;
  	sp.sub = sp.n.subscribe("/data/imu", 1000, sp.imuCallback);

	ros::Subscriber sub = n.subscribe("/data/imu", 1000, imuCallback);

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
	int a = 0;
}