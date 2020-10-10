#include "ros/ros.h"
#include "rcs_msg_wrapper/dwdx.h"
#include "geometry_msgs/TwistStamped.h"
geometry_msgs::TwistStamped tw_msgs;
void tf_ros_dwdx(const rcs_msg_wrapper::dwdxConstPtr& msg){
    tw_msgs.header.frame_id = "esr";
    tw_msgs.twist.linear.x = msg->global_vx * 0.01;
    tw_msgs.twist.linear.y = msg->global_vy * 0.01;
    tw_msgs.twist.linear.z = msg->global_vz * 0.01;
    tw_msgs.twist.angular.x = msg->global_wx * 0.01;
    tw_msgs.twist.angular.y = msg->global_wy * 0.01;
    tw_msgs.twist.angular.z = msg->global_wz * 0.01;

}
int main(int argc, char** argv){
    ros::init(argc, argv, "tfdwdx");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ros_dwdx", 10, tf_ros_dwdx);
    ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/xsens/twist", 100);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        pub.publish(tw_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
