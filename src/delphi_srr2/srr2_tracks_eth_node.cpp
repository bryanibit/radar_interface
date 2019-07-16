/**
 *  This file is a part of radar_interface.
 *
 *  Copyright (C) 2018 Juraj Persic, University of Zagreb Faculty of Electrical
 Engineering and Computing

 *  radar_interface is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "radar_interface/delphi_srr2/srr2_tracks_eth.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "srr2_tracks_eth");
  ros::NodeHandle n("~");

  std::string ip_address;
  std::string name;
  std_msgs::Header header;

  int port;

  if (!n.getParam("ip", ip_address)) {
    // Default value if ip argument is missing
    ip_address = SRR2_IP_ADDR_L;
  }
  if (!n.getParam("port", port)) {
    // Default value if ip argument is missing
    port = SRR2_PORT_DSP;
  }
  if (!n.getParam("name", name)) {
    // Default value if position is missing
    name = "srr2";
  } else {
    name = "srr2_" + name;
  }
  header.frame_id = name;
  ROS_INFO("ip: %s , port: %d , name: %s", ip_address.c_str(), port,
           name.c_str());

  ros::Publisher tcp_raw_pub =
      n.advertise<radar_interface::TCPFrame>("tcp_raw", 1000);
  ros::Publisher tracks_pub =
      n.advertise<radar_interface::RadarTrackArray>("tracks", 1000);
  ros::Publisher markers_pub =
      n.advertise<visualization_msgs::MarkerArray>("markers", 1000);
  ros::Publisher vehicle_info_pub =
      n.advertise<radar_interface::VehicleInfo>("vehicle_info", 1000);
  ros::Publisher alignment_info_pub =
      n.advertise<radar_interface::AlignmentInfoSRR2>("alignment_info", 1000);

  radar_interface::TCPFrame tcp_frame_msg;
  radar_interface::RadarTrackArray tracks_msg;
  tracks_msg.tracks.resize(SRR2_MAX_TRCK_NUM);
  radar_interface::VehicleInfo vehicle_info_msg;
  radar_interface::AlignmentInfoSRR2 alignment_info_msg;

  RadarMarkers radar_markers;
  radar_markers.marker_num_ = SRR2_MAX_TRCK_NUM;
  radar_markers.marker_namespace_ = name;
  radar_markers.initializePublisher(&n);

  TCPInterface tcp_interface;

  tcp_frame_msg.address = ip_address;
  tcp_frame_msg.port = port;
  tcp_frame_msg.size = SRR2_XCP_PAYLOAD_SIZE;
  tcp_frame_msg.data.resize(SRR2_XCP_PAYLOAD_SIZE);

  int status = tcp_interface.open(ip_address.c_str(), port);

  unsigned char xcpMsgBuf[SRR2_XCP_PAYLOAD_SIZE];
  size_t bytes_read = SRR2_XCP_PAYLOAD_SIZE;

  // Radar info
  unsigned long int scanId;
  unsigned long int time_stamp;
  unsigned long dsp_version;
  unsigned long dsp1, dsp2, dsp3;
  // Track info
  unsigned long int track_count;
  signed short int track_x_pos;
  signed short int track_x_vel;
  signed short int track_x_acc;
  signed short int track_y_pos;
  signed short int track_y_vel;
  signed short int track_y_acc;
  unsigned char track_status;
  unsigned char is_movable;
  // Vehicle Info
  signed int steering_angle;
  signed int yaw_rate;
  signed int speed;
  bool is_reverse;
  uint8_t turn_signal;
  uint8_t steering_angle_sign;
  signed int yaw_rate_raw;
  uint8_t yaw_rate_raw_qf;
  uint8_t yaw_rate_qf;
  // Alignment info
  uint8_t in_command;
  unsigned short int in_updated_needed;
  signed int in_angle;
  unsigned int out_status;
  signed int out_angle;
  unsigned int out_state;
  unsigned short int out_updates_completed;

  while (ros::ok()) {

    // tcp_interface.read_exactly();

    status = tcp_interface.read_exactly(xcpMsgBuf, SRR2_XCP_PAYLOAD_SIZE,
                                        bytes_read, SRR2_ETH_TIMEOUT);

    if (status != 0) {
      ROS_INFO("Status: %d\n", status);
    } else {
      header.stamp = ros::Time::now();

      tcp_interface.parse_value(&scanId, xcpMsgBuf, SRR2_SCAN_INDEX_OFFSET,
                                SRR2_SCAN_INDEX_SIZE);
      tcp_interface.parse_value(&time_stamp, xcpMsgBuf, SRR2_TIME_STAMP_OFFSET,
                                SRR2_TIME_STAMP_SIZE);

      /* Uncomment to get firmware versoin
      tcp_interface.parse_value(&dsp1, xcpMsgBuf, SRR2_DSP_VER_1_OFFSET,
      SRR2_DSP_VER_1_SIZE);
      tcp_interface.parse_value(&dsp2, xcpMsgBuf, SRR2_DSP_VER_2_OFFSET,
      SRR2_DSP_VER_2_SIZE);
      tcp_interface.parse_value(&dsp3, xcpMsgBuf, SRR2_DSP_VER_3_OFFSET,
      SRR2_DSP_VER_3_SIZE);
      */

      // ROS_INFO("Time stamp: %d", time_stamp);
      // ROS_INFO("Scan ID: %d", scanId);

      // Parse and publish track information
      for (size_t i = 0; i < 64; i++) {
        // clang-format off
      tcp_interface.parse_value(&track_x_pos,   xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_X_POS_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_X_POS_SIZE);
      tcp_interface.parse_value(&track_x_vel,   xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_X_VEL_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_X_VEL_SIZE);
      tcp_interface.parse_value(&track_x_acc,   xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_X_ACC_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_X_ACC_SIZE);
      tcp_interface.parse_value(&track_y_pos,   xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_Y_POS_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_Y_POS_SIZE);
      tcp_interface.parse_value(&track_y_vel,   xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_Y_VEL_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_Y_VEL_SIZE);
      tcp_interface.parse_value(&track_y_acc,   xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_Y_ACC_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_Y_ACC_SIZE);
      tcp_interface.parse_value(&track_status,  xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_STATUS_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_STATUS_SIZE);
      tcp_interface.parse_value(&is_movable, xcpMsgBuf, SRR2_TRCK_OFFSET + SRR2_TRCK_STATIONARY_OFFSET + SRR2_TRCK_SIZE*i, SRR2_TRCK_STATIONARY_SIZE);
        // clang-format on
        // if (track_x_pos != 0) {
        //   std::cout << track_x_pos << "," << track_x_vel << "," <<
        //   track_x_acc
        //             << "," << track_y_pos << "," << track_y_vel << ","
        //             << track_y_acc << std::endl;
        //   std::cout << (signed int)track_x_pos << "," << track_x_vel << ","
        //             << track_x_acc << "," << track_y_pos << "," <<
        //             track_y_vel
        //             << "," << track_y_acc << std::endl;
        // }
        tracks_msg.tracks[i].id = i;
        tracks_msg.tracks[i].pos.x = track_x_pos / SRR2_TRCK_POS_SCALE;
        tracks_msg.tracks[i].pos.y = track_y_pos / SRR2_TRCK_POS_SCALE;
        tracks_msg.tracks[i].vel.x = track_x_vel / SRR2_TRCK_VEL_SCALE;
        tracks_msg.tracks[i].vel.y = track_y_vel / SRR2_TRCK_VEL_SCALE;
        tracks_msg.tracks[i].acc.x = track_x_acc / SRR2_TRCK_ACC_SCALE;
        tracks_msg.tracks[i].acc.y = track_y_acc / SRR2_TRCK_ACC_SCALE;
        tracks_msg.tracks[i].status = track_status;
        tracks_msg.tracks[i].is_movable = is_movable;
      }
      tracks_msg.header = header;
      tracks_pub.publish(tracks_msg);

      // Parse vehicle information
      tcp_interface.parse_value(&steering_angle, xcpMsgBuf,
                                SRR2_STEERING_ANGLE_OFFSET,
                                SRR2_STEERING_ANGLE_SIZE);
      tcp_interface.parse_value(&yaw_rate, xcpMsgBuf, SRR2_YAW_RATE_OFFSET,
                                SRR2_YAW_RATE_SIZE);
      tcp_interface.parse_value(&speed, xcpMsgBuf, SRR2_SPEED_OFFSET,
                                SRR2_SPEED_SIZE);
      tcp_interface.parse_value(&is_reverse, xcpMsgBuf, SRR2_IS_REVERSE_OFFSET,
                                SRR2_IS_REVERSE_SIZE);
      tcp_interface.parse_value(&turn_signal, xcpMsgBuf,
                                SRR2_TURN_SIGNAL_OFFSET, SRR2_TURN_SIGNAL_SIZE);
      tcp_interface.parse_value(&steering_angle_sign, xcpMsgBuf,
                                SRR2_STEERING_ANGLE_SIGN_OFFSET,
                                SRR2_STEERING_ANGLE_SIGN_SIZE);
      tcp_interface.parse_value(&yaw_rate_raw, xcpMsgBuf,
                                SRR2_YAW_RATE_RAW_OFFSET,
                                SRR2_YAW_RATE_RAW_SIZE);
      tcp_interface.parse_value(&yaw_rate_raw_qf, xcpMsgBuf,
                                SRR2_YAW_RATE_RAW_QF_OFFSET,
                                SRR2_YAW_RATE_RAW_QF_SIZE);
      tcp_interface.parse_value(&yaw_rate_qf, xcpMsgBuf,
                                SRR2_YAW_RATE_QF_OFFSET, SRR2_YAW_RATE_QF_SIZE);

      vehicle_info_msg.steering_angle =
          steering_angle / SRR2_STEERING_ANGLE_SCALE;
      vehicle_info_msg.speed = speed / SRR2_SPEED_SCALE;
      vehicle_info_msg.yaw_rate = yaw_rate / SRR2_YAW_RATE_SCALE;
      vehicle_info_msg.yaw_rate_raw = yaw_rate_raw / SRR2_YAW_RATE_SCALE;

      vehicle_info_msg.yaw_rate = yaw_rate / SRR2_YAW_RATE_SCALE;
      vehicle_info_msg.is_reverse = is_reverse;
      vehicle_info_msg.turn_signal = turn_signal;
      vehicle_info_msg.steering_angle_sign = steering_angle_sign;
      vehicle_info_msg.yaw_rate_qf = yaw_rate_qf;
      vehicle_info_msg.yaw_rate_raw_qf = yaw_rate_raw_qf;

      vehicle_info_msg.header = header;
      vehicle_info_pub.publish(vehicle_info_msg);

      // Parse alignment info

      tcp_interface.parse_value(&in_command, xcpMsgBuf,
                                SRR2_ALIGN_INPUT_COMMAND_OFFSET,
                                SRR2_ALIGN_INPUT_COMMAND_SIZE);
      tcp_interface.parse_value(&in_updated_needed, xcpMsgBuf,
                                SRR2_ALIGN_INPUT_UPDATED_NEEDED_OFFSET,
                                SRR2_ALIGN_INPUT_UPDATED_NEEDED_SIZE);
      tcp_interface.parse_value(&in_angle, xcpMsgBuf,
                                SRR2_ALIGN_INPUT_ANGLE_OFFSET,
                                SRR2_ALIGN_INPUT_ANGLE_SIZE);

      tcp_interface.parse_value(&out_status, xcpMsgBuf,
                                SRR2_ALIGN_OUTPUT_STATUS_OFFSET,
                                SRR2_ALIGN_OUTPUT_STATUS_SIZE);
      tcp_interface.parse_value(&out_angle, xcpMsgBuf,
                                SRR2_ALIGN_OUTPUT_ANGLE_OFFSET,
                                SRR2_ALIGN_OUTPUT_ANGLE_SIZE);
      tcp_interface.parse_value(&out_state, xcpMsgBuf,
                                SRR2_ALIGN_OUTPUT_STATE_OFFSET,
                                SRR2_ALIGN_OUTPUT_STATE_SIZE);
      tcp_interface.parse_value(&out_updates_completed, xcpMsgBuf,
                                SRR2_ALIGN_OUTPUT_UPDATED_COMPLETED_OFFSET,
                                SRR2_ALIGN_OUTPUT_UPDATED_COMPLETED_SIZE);

      alignment_info_msg.in_command = in_command;
      alignment_info_msg.in_updated_needed = in_updated_needed;
      alignment_info_msg.in_angle = in_angle / SRR2_ALIGN_ANGLE_SCALE;
      alignment_info_msg.out_status = out_status;
      alignment_info_msg.out_angle = out_angle / SRR2_ALIGN_ANGLE_SCALE;
      alignment_info_msg.out_state = out_state;
      alignment_info_msg.out_updates_completed = out_updates_completed;

      alignment_info_msg.header = header;
      alignment_info_pub.publish(alignment_info_msg);

      // Raw TCP frame
      for (size_t i = 0; i < SRR2_XCP_PAYLOAD_SIZE; i++) {
        tcp_frame_msg.data[i] = xcpMsgBuf[i];
      }
      tcp_frame_msg.header = header;
      tcp_raw_pub.publish(tcp_frame_msg);
    }
  }
  return 0;
}