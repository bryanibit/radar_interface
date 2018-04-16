#include "radar_interface/esr_targets_eth.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "esr_targets_eth");

  ros::NodeHandle n("~");
  ros::Publisher chatter_pub =
      n.advertise<radar_interface::TCPFrame>("tcp_raw", 1000);
  radar_interface::TCPFrame tcp_frame_msg;

  TCPInterface tcp_interface;
  radar_interface::RadarTrack track;

  std::string ip_address;
  int port;

  if (!n.getParam("ip", ip_address)) {
    // Default value if ip argument is missing
    ip_address = "192.168.1.26";
  }
  if (!n.getParam("port", port)) {
    // Default value if ip argument is missing
    port = 5555;
  }
  ROS_INFO("ip: %s ,  port: %d", ip_address.c_str(), port);

  int status = tcp_interface.open(ip_address.c_str(), port);

  ROS_INFO("%d", status);

  unsigned char xcpMsgBuf[ESR_XCP_PAYLOAD_SIZE];
  size_t bytes_read = ESR_XCP_PAYLOAD_SIZE;

  // status = tcp_interface.open(ip_address.c_str(), port);

  //  status = tcp_interface.read( xcpMsgBuf, ESR_XCP_PAYLOAD_SIZE,
  // bytes_read);

  unsigned long scanId;
  unsigned long time_stamp;
  unsigned long target_count;
  unsigned long target_range[ESR_MAX_TARGET_NUM];
  unsigned long dsp_version;
  unsigned long dsp1, dsp2, dsp3;

  while (ros::ok()) {

    // tcp_interface.read_exactly();

    status =
        tcp_interface.read_exactly(xcpMsgBuf, ESR_XCP_PAYLOAD_SIZE, bytes_read);

    tcp_interface.parse_value(&scanId, xcpMsgBuf, ESR_SCAN_INDEX_OFFSET,
                              ESR_SCAN_INDEX_SIZE);
    tcp_interface.parse_value(&time_stamp, xcpMsgBuf, ESR_TIME_STAMP_OFFSET,
                              ESR_TIME_STAMP_SIZE);
            tcp_interface.parse_value(&target_count, xcpMsgBuf, ESR_TGT_RPT_CNT_OFFSET, 1);
    // 1 istead of ESR_TGT_RPT_CNT_SIZE because the higher byte is sometimes
    // corrupted

    // std::cout << (int)xcpMsgBuf[ESR_TGT_RPT_CNT_OFFSET] << ","
    //           << (int)xcpMsgBuf[ESR_TGT_RPT_CNT_OFFSET + 1] <<std::endl;

    // target_count =
    //     tcp_interface.parse_array(xcpMsgBuf, target_range,
    //     ESR_TGT_RNG_OFFSET,<s
    //                               ESR_TGT_RNG_SIZE, ESR_MAX_TARGET_NUM);

    tcp_interface.parse_value(&dsp1, xcpMsgBuf, ESR_DSP_VER_1_OFFSET,
                              ESR_DSP_VER_1_SIZE);
    tcp_interface.parse_value(&dsp2, xcpMsgBuf, ESR_DSP_VER_2_OFFSET,
                              ESR_DSP_VER_2_SIZE);
    tcp_interface.parse_value(&dsp3, xcpMsgBuf, ESR_DSP_VER_3_OFFSET,
                              ESR_DSP_VER_3_SIZE);
    // std::cout << "DSP Version: " << dsp1 << "." << dsp2 << "." << dsp3 <<
    // std::endl;
    // std::cout << "DSP Version: " << dsp1 << "." << dsp2 << "." << dsp3 <<
    // std::endl;
    // ROS_INFO("Target cound: %d", time_stamp);
    ROS_INFO("Target count: %d", (int)target_count);

    // std::cout << "\n";
    // for (size_t i = 0; i < 64; i++)
    // {
    //   std::cout << target_range[i]/128 << "," ;
    // }
    // std::cout << "\n";

    // status = tcp_interface.read_exactly(xcpMsgBuf, ESR_XCP_PAYLOAD_SIZE,
    //                             bytes_read);
    // ROS_INFO("%d", status);
    // chatter_pub.publish(msg);
  }

  return 0;
}