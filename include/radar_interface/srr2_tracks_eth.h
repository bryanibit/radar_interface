#include "radar_interface/tcp_interface.h"
#include <cstdio>
// #include "network_interface/TCPFrame.h"
// #include "network_interface/network_interface.h"
#include "radar_interface/RadarTrack.h"
#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/TCPFrame.h"
#include "radar_msgs/RadarTrack.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

const int SRR2_PORT_ARM = 5555;
const int SRR2_PORT_DSP = 55555;
const std::string SRR2_IP_ADDR_L = "169.254.145.71";
const std::string SRR2_IP_ADDR_R = "169.254.145.72";

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;
const unsigned int SRR2_XCP_PAYLOAD_SIZE = 1240;

const unsigned int SRR2_DSP_VER_1_OFFSET = 28;
const unsigned int SRR2_DSP_VER_1_SIZE = 4;
const unsigned int SRR2_DSP_VER_2_OFFSET = 32;
const unsigned int SRR2_DSP_VER_2_SIZE = 4;
const unsigned int SRR2_DSP_VER_3_OFFSET = 36;
const unsigned int SRR2_DSP_VER_3_SIZE = 4;

const unsigned int SRR2_SCAN_INDEX_OFFSET = 48;
const unsigned int SRR2_SCAN_INDEX_SIZE = 4;
const unsigned int SRR2_TIME_STAMP_OFFSET = 52;
const unsigned int SRR2_TIME_STAMP_SIZE = 4;

const unsigned int SRR2_TRCK_OFFSET = 68;
const unsigned int SRR2_TRCK_SIZE = 14;
// OFFSETS FOR TRACK DATA DEFINED WITHIN ONE TRACK
const unsigned int SRR2_TRCK_X_POS_OFFSET = 0;
const unsigned int SRR2_TRCK_X_VEL_OFFSET = 2;
const unsigned int SRR2_TRCK_X_ACC_OFFSET = 4;
const unsigned int SRR2_TRCK_Y_POS_OFFSET = 6;
const unsigned int SRR2_TRCK_Y_VEL_OFFSET = 8;
const unsigned int SRR2_TRCK_Y_ACC_OFFSET = 10;
const unsigned int SRR2_TRCK_X_POS_SIZE = 2;
const unsigned int SRR2_TRCK_X_VEL_SIZE = 2;
const unsigned int SRR2_TRCK_X_ACC_SIZE = 2;
const unsigned int SRR2_TRCK_Y_POS_SIZE = 2;
const unsigned int SRR2_TRCK_Y_VEL_SIZE = 2;
const unsigned int SRR2_TRCK_Y_ACC_SIZE = 2;
const unsigned int SRR2_TRCK_STATUS_OFFSET = 12;
const unsigned int SRR2_TRCK_STATUS_SIZE = 1;
const unsigned int SRR2_TRCK_STATIONARY_OFFSET = 13;
const unsigned int SRR2_TRCK_STATIONARY_SIZE = 1;

const unsigned int SRR2_MAX_TRCK_NUM = 64;