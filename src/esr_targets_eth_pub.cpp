#include "network_interface/TCPFrame.h"
#include "network_interface/network_interface.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "esr_targets_eth");

  ros::NodeHandle n;
  ros::Publisher chatter_pub =
      n.advertise<network_interface::TCPFrame>("tcp_raw", 1000);
  network_interface::TCPFrame tcp_frame_msg;


  AS::Network::TCPInterface tcp_interface;


  // ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    //     ROS_INFO("%s", msg.data.c_str());

    // chatter_pub.publish(msg);

    // ros::spinOnce();

    // loop_rate.sleep();
    // ++count;
  }

  return 0;
}