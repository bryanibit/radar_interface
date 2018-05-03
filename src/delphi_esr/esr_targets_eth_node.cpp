#include "radar_interface/delphi_esr/esr_targets_eth.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "esr_targets_eth");
  ros::NodeHandle n("~");

  TCPInterface tcp_interface;

  std::string ip_address, name;
  int port;

  if (!n.getParam("ip", ip_address)) {
    // Default value if ip argument is missing
    ip_address = "192.168.1.26";
  }
  if (!n.getParam("port", port)) {
    // Default value if ip argument is missing
    port = 5555;
  }
  if (!n.getParam("name", name)) {
    // Default value if ip argument is missing
    name = "esr";
  } else {
    name = "esr_" + name;
  }
  ROS_INFO("ip: %s ,  port: %d, name: %s", ip_address.c_str(), port,
           name.c_str());

  ros::Publisher tcp_raw_pub =
      n.advertise<radar_interface::TCPFrame>("tcp_raw", 1000);
  ros::Publisher target_pub =
      n.advertise<radar_interface::RadarTargetArray>("targets", 1000);
  ros::Publisher markers_pub =
      n.advertise<visualization_msgs::MarkerArray>("markers", 1000);
  ros::Publisher vehicle_info_pub =
      n.advertise<radar_interface::VehicleInfo>("vehicle_info", 1000);

  int status = tcp_interface.open(ip_address.c_str(), port);

  unsigned char xcpMsgBuf[ESR_XCP_PAYLOAD_SIZE];
  size_t bytes_read = ESR_XCP_PAYLOAD_SIZE;

  radar_interface::TCPFrame tcp_frame_msg;
  radar_interface::RadarTargetArray target_msg;
  target_msg.targets.resize(ESR_MAX_TARGET_NUM);
  radar_interface::VehicleInfo vehicle_info_msg;
  std_msgs::Header header;
  header.frame_id = name;
  tcp_frame_msg.address = ip_address;
  tcp_frame_msg.port = port;
  tcp_frame_msg.size = ESR_XCP_PAYLOAD_SIZE;
  tcp_frame_msg.data.resize(ESR_XCP_PAYLOAD_SIZE);

  RadarMarkers radar_markers;
  radar_markers.marker_array_.markers.resize(ESR_MAX_TARGET_NUM);
  radar_markers.marker_namespace_ = name;
  radar_markers.initializePublisher(&n);
  radar_markers.colormap.setColormap(ESR_TGT_MIN_RCS, ESR_TGT_MAX_RCS);

  unsigned long scanId;
  unsigned long time_stamp;
  unsigned long target_count;
  unsigned long target_range[ESR_MAX_TARGET_NUM];
  unsigned long dsp_version;
  unsigned long dsp1, dsp2, dsp3;
  int16_t speed, yaw_rate;
  int16_t range[ESR_MAX_TARGET_NUM], range_rate[ESR_MAX_TARGET_NUM],
      azimuth[ESR_MAX_TARGET_NUM], rcs[ESR_MAX_TARGET_NUM];

  while (ros::ok()) {
    status =
        tcp_interface.read_exactly(xcpMsgBuf, ESR_XCP_PAYLOAD_SIZE, bytes_read);

    header.stamp = ros::Time::now();

    tcp_interface.parse_value(&scanId, xcpMsgBuf, ESR_SCAN_INDEX_OFFSET,
                              ESR_SCAN_INDEX_SIZE);
    tcp_interface.parse_value(&time_stamp, xcpMsgBuf, ESR_TIME_STAMP_OFFSET,
                              ESR_TIME_STAMP_SIZE);

    //
    // Parsing target information; publishing targets and markers
    //

    // target count contraty to TCP datasheet
    tcp_interface.parse_value(&target_count, xcpMsgBuf, ESR_TGT_RPT_CNT_OFFSET,
                              1);
    // 1 istead of ESR_TGT_RPT_CNT_SIZE because the higher byte is sometimes
    // corrupted

    tcp_interface.parse_array(range, xcpMsgBuf, ESR_TGT_RNG_OFFSET,
                              ESR_TGT_RNG_SIZE, ESR_MAX_TARGET_NUM);
    tcp_interface.parse_array(range_rate, xcpMsgBuf, ESR_TGT_RNG_RATE_OFFSET,
                              ESR_TGT_RNG_RATE_SIZE, ESR_MAX_TARGET_NUM);
    tcp_interface.parse_array(azimuth, xcpMsgBuf, ESR_TGT_AZIM_OFFSET,
                              ESR_TGT_AZIM_SIZE, ESR_MAX_TARGET_NUM);
    tcp_interface.parse_array(rcs, xcpMsgBuf, ESR_TGT_RCS_OFFSET,
                              ESR_TGT_RCS_SIZE, ESR_MAX_TARGET_NUM);

    target_msg.header = header;

    for (size_t i = 0; i < ESR_MAX_TARGET_NUM; i++) {

      target_msg.targets[i].range = range[i];
      target_msg.targets[i].range_rate = range_rate[i];
      target_msg.targets[i].azimuth = azimuth[i];
      target_msg.targets[i].rcs = rcs[i];
      if (i < target_count) {
        target_msg.targets[i].status = 1;
      } else if (i >= target_count && range[i] != 0.0) {
        target_msg.targets[i].status = -1;
      } else {
        target_msg.targets[i].status = 0;
      }
    }

    target_pub.publish(target_msg);
    radar_markers.updateTargetMarkers(target_msg);

    //
    // Parsing and publishing vehicle information
    //

    tcp_interface.parse_value(&speed, xcpMsgBuf, ESR_HOST_SPEED_OFFSET,
                              ESR_HOST_SPEED_SIZE);
    tcp_interface.parse_value(&yaw_rate, xcpMsgBuf, ESR_HOST_YAW_RATE_OFFSET,
                              ESR_HOST_YAW_RATE_SIZE);

    vehicle_info_msg.speed = speed / ESR_HOST_SPEED_SCALE;
    vehicle_info_msg.yaw_rate = yaw_rate / ESR_HOST_YAW_RATE_SCALE;

    vehicle_info_msg.header = header;
    vehicle_info_pub.publish(vehicle_info_msg);

    /* Uncomment following lines to get DSP version
    tcp_interface.parse_value(&dsp1, xcpMsgBuf, ESR_DSP_VER_1_OFFSET,
                              ESR_DSP_VER_1_SIZE);
    tcp_interface.parse_value(&dsp2, xcpMsgBuf, ESR_DSP_VER_2_OFFSET,
                              ESR_DSP_VER_2_SIZE);
    tcp_interface.parse_value(&dsp3, xcpMsgBuf, ESR_DSP_VER_3_OFFSET,
                              ESR_DSP_VER_3_SIZE);
    std::cout << "DSP Version: " << dsp1 << "." << dsp2 << "." << dsp3 <<
    std::endl;
    */
  }

  return 0;
}