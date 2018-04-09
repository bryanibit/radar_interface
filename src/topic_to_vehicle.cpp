#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

/**
 * This node listens to topic provided by the argument and sends the vehicle
   motion information to the can interface
 */
void vehicleCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char *argv[]) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line.
   * For programmatic remappings you can use a different version of init() which
   * takes
   * remappings directly, but for most command-line programs, passing argc and
   * argv is
   * the easiest way to do it.  The third argument to init() is the name of the
   * node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "topic_to_vehicle");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n("~");
  std::string can_device;
  std::string subscribed_topic;
  bool toSubscribe = false;

  toSubscribe = n.getParam("subscribed_topic", subscribed_topic);

  if (toSubscribe) {
    ROS_INFO("Subscribed to geometry_msgs::twist topic: [%s]",
             subscribed_topic.c_str());
    ros::Subscriber sub = n.subscribe(subscribed_topic, 1000, vehicleCallback);
  } else {
    ROS_INFO("Publishing zero velocities to [%s]", can_device.c_str());
  }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).
   * ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}