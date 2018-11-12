#include "geometry_msgs/Twist.h"
#include "radar_interface/can_tools.h"
#include "socketcan_bridge/topic_to_socketcan.h"
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <ros/ros.h>
#include <socketcan_interface/interface.h>
#include <socketcan_interface/socketcan.h>

#define RAD_TO_DEG 57.295779513

can_tools::CANParseValueInfo VEH_VEL = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 0,
    .END_BYTE = 1,
    .SHIFT = 5,
    .SCALE = 0.0625,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 127.9375,
    .MASK = {0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo VEH_VEL_DIR = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 1,
    .END_BYTE = 1,
    .SHIFT = 4,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

can_tools::CANParseValueInfo VEH_YAW_RATE = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 1,
    .END_BYTE = 2,
    .SHIFT = 0,
    .SCALE = 0.0625,
    .OFFSET = 0,
    .MIN = -128,
    .MAX = 127.9375,
    .MASK = {0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00}};
can_tools::CANParseValueInfo VEH_YAW_RATE_VALID = {
    .MSG_ID = 0x4F0,
    .START_BYTE = 3,
    .END_BYTE = 3,
    .SHIFT = 7,
    .SCALE = 1,
    .OFFSET = 0,
    .MIN = 0,
    .MAX = 1,
    .MASK = {0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00}};

class VehicleToESR : public socketcan_bridge::TopicToSocketCAN {
public:
  VehicleToESR(ros::NodeHandle *nh, ros::NodeHandle *nh_param,
               can::DriverInterfaceSharedPtr driver);
  void sendCanFrame(const ros::TimerEvent &event);
  virtual ~VehicleToESR();

private:
  ros::Subscriber twist_topic_;
  void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void setFrameProperties(can::Frame *frame);
  geometry_msgs::Twist twist_;
  can::DriverInterfaceSharedPtr driver_interface_;
  can::Frame frame_vehicle1_;
};