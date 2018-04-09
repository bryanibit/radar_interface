#include "radar_interface/srr2_tracks_eth.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "srr2_tracks_eth");

  ros::NodeHandle n("~");
  ros::Publisher tcp_raw =
      n.advertise<radar_interface::TCPFrame>("tcp_raw", 1000);
  radar_interface::TCPFrame tcp_frame_msg;

  TCPInterface tcp_interface;
  radar_interface::RadarTrack track[SRR2_MAX_TRCK_NUM];
  radar_interface::RadarTrackArray tracks_msg;
  tracks_msg.tracks.resize(SRR2_MAX_TRCK_NUM);

  std::string ip_address;
  int port;

  if (!n.getParam("ip", ip_address)) {
    // Default value if ip argument is missing
    ip_address = SRR2_IP_ADDR_L;
  }
  if (!n.getParam("port", port)) {
    // Default value if ip argument is missing
    port = SRR2_PORT_DSP;
  }
  ROS_INFO("ip: %s ,  port: %d", ip_address.c_str(), port);

  int status = tcp_interface.open(ip_address.c_str(), port);

  ROS_INFO("%d", status);

  unsigned char xcpMsgBuf[SRR2_XCP_PAYLOAD_SIZE];
  size_t bytes_read = SRR2_XCP_PAYLOAD_SIZE;

  // status = tcp_interface.open(ip_address.c_str(), port);

  //  status = tcp_interface.read( xcpMsgBuf, SRR2_XCP_PAYLOAD_SIZE,
  // bytes_read);

  unsigned long int scanId;
  unsigned long int time_stamp;
  unsigned long int track_count;
  signed short int track_x_pos;
  signed short int track_x_vel;
  signed short int track_x_acc;
  signed short int track_y_pos;
  signed short int track_y_vel;
  signed short int track_y_acc;
  unsigned long dsp_version;
  unsigned long dsp1, dsp2, dsp3;
  while (ros::ok()) {

    // tcp_interface.read_exactly();

    status = tcp_interface.read_exactly(xcpMsgBuf, SRR2_XCP_PAYLOAD_SIZE,
                                        bytes_read);

    tcp_interface.parse_value(&scanId, xcpMsgBuf, SRR2_SCAN_INDEX_OFFSET,
                              SRR2_SCAN_INDEX_SIZE);
    tcp_interface.parse_value(&time_stamp, xcpMsgBuf, SRR2_TIME_STAMP_OFFSET,
                              SRR2_TIME_STAMP_SIZE);
    tcp_interface.parse_value(&dsp1, xcpMsgBuf, SRR2_DSP_VER_1_OFFSET,
                              SRR2_DSP_VER_1_SIZE);
    tcp_interface.parse_value(&dsp2, xcpMsgBuf, SRR2_DSP_VER_2_OFFSET,
                              SRR2_DSP_VER_2_SIZE);
    tcp_interface.parse_value(&dsp3, xcpMsgBuf, SRR2_DSP_VER_3_OFFSET,
                              SRR2_DSP_VER_3_SIZE);

    // ROS_INFO("Time stamp: %d", time_stamp);
    // ROS_INFO("Scan ID: %d", scanId);

    // std::cout << "\n";
    for (size_t i = 0; i < 64; i++) {
      // clang-format off
      tcp_interface.parse_value(&track_x_pos, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_X_POS_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_X_POS_SIZE);
      tcp_interface.parse_value(&track_x_vel, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_X_VEL_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_X_VEL_SIZE);
      tcp_interface.parse_value(&track_x_acc, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_X_ACC_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_X_ACC_SIZE);
      tcp_interface.parse_value(&track_y_pos, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_Y_POS_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_Y_POS_SIZE);
      tcp_interface.parse_value(&track_y_vel, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_Y_VEL_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_Y_VEL_SIZE);
      tcp_interface.parse_value(&track_y_acc, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_Y_ACC_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_Y_ACC_SIZE);
      if (track_x_pos !=0 )
      {
      std::cout << track_x_pos <<","<< track_x_vel <<","<< track_x_acc <<","<< track_y_pos <<","<< track_y_vel <<","<<  track_y_acc << std::endl;
      std::cout << (signed int) track_x_pos <<","<< track_x_vel <<","<< track_x_acc <<","<< track_y_pos <<","<< track_y_vel <<","<<  track_y_acc << std::endl;
      }

      // clang-format on
    }
    // std::cout << "\n";

    // status = tcp_interface.read_exactly(xcpMsgBuf, SRR2_XCP_PAYLOAD_SIZE,
    //                             bytes_read);
    // ROS_INFO("%d", status);
    // chatter_pub.publish(msg);
  }

  return 0;
}