#include "radar_interface/radar_visualization.h"
#include "radar_interface/tcp_interface.h"
#include <cstdio>
#include <math.h>
#include "radar_interface/RadarTrack.h"
#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/TCPFrame.h"
#include "radar_interface/VehicleInfo.h"
#include "radar_interface/AlignmentInfoSRR2.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"


#include <sstream>

const int SRR2_PORT_ARM = 5555;
const int SRR2_PORT_DSP = 55555;
const std::string SRR2_IP_ADDR_L = "169.254.145.71";
const std::string SRR2_IP_ADDR_R = "169.254.145.72";
const int SRR2_ETH_TIMEOUT = 1000;

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

///////////////////////////////////////////////////////////////////////////////
// Track information
//
const unsigned int SRR2_TRCK_OFFSET = 68;
const unsigned int SRR2_TRCK_SIZE = 14;
// OFFSETS FOR TRACK DATA DEFINED WITHIN ONE TRACK
float SRR2_TRCK_POS_SCALE = 128.0;
float SRR2_TRCK_VEL_SCALE = 128.0;
float SRR2_TRCK_ACC_SCALE = 1024.0;
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

///////////////////////////////////////////////////////////////////////////////
// Vehicle information
//
float SRR2_YAW_RATE_SCALE = pow(2, 16) * 1.0;
float SRR2_STEERING_ANGLE_SCALE = pow(2, 16) * 1.0;
float SRR2_SPEED_SCALE = pow(2, 16) * 1.0;
const unsigned int SRR2_STEERING_ANGLE_OFFSET = 1040;
const unsigned int SRR2_STEERING_ANGLE_SIZE = 4;
const unsigned int SRR2_YAW_RATE_OFFSET = 1044;
const unsigned int SRR2_YAW_RATE_SIZE = 4;
const unsigned int SRR2_SPEED_OFFSET = 1048;
const unsigned int SRR2_SPEED_SIZE = 4;
const unsigned int SRR2_IS_REVERSE_OFFSET = 1052;
const unsigned int SRR2_IS_REVERSE_SIZE = 1;
const unsigned int SRR2_TURN_SIGNAL_OFFSET = 1053;
const unsigned int SRR2_TURN_SIGNAL_SIZE = 1;
const unsigned int SRR2_STEERING_ANGLE_SIGN_OFFSET = 1054;
const unsigned int SRR2_STEERING_ANGLE_SIGN_SIZE = 1;
const unsigned int SRR2_YAW_RATE_RAW_OFFSET = 1056;
const unsigned int SRR2_YAW_RATE_RAW_SIZE = 4;
const unsigned int SRR2_YAW_RATE_RAW_QF_OFFSET = 1060;
const unsigned int SRR2_YAW_RATE_RAW_QF_SIZE = 1;
const unsigned int SRR2_YAW_RATE_QF_OFFSET = 1061;
const unsigned int SRR2_YAW_RATE_QF_SIZE = 1;

///////////////////////////////////////////////////////////////////////////////
// Alignment info
//
float SRR2_ALIGN_ANGLE_SCALE = pow(2, 26) * 1.0;
const unsigned int SRR2_ALIGN_INPUT_COMMAND_OFFSET = 976;
const unsigned int SRR2_ALIGN_INPUT_COMMAND_SIZE = 1;
const unsigned int SRR2_ALIGN_INPUT_UPDATED_NEEDED_OFFSET = 977;
const unsigned int SRR2_ALIGN_INPUT_UPDATED_NEEDED_SIZE = 2;
const unsigned int SRR2_ALIGN_INPUT_ANGLE_OFFSET = 979;
const unsigned int SRR2_ALIGN_INPUT_ANGLE_SIZE = 4;

const unsigned int SRR2_ALIGN_OUTPUT_STATUS_OFFSET = 996;
const unsigned int SRR2_ALIGN_OUTPUT_STATUS_SIZE = 4;
const unsigned int SRR2_ALIGN_OUTPUT_ANGLE_OFFSET = 1000;
const unsigned int SRR2_ALIGN_OUTPUT_ANGLE_SIZE = 4;
const unsigned int SRR2_ALIGN_OUTPUT_STATE_OFFSET = 1004;
const unsigned int SRR2_ALIGN_OUTPUT_STATE_SIZE = 4;
const unsigned int SRR2_ALIGN_OUTPUT_UPDATED_COMPLETED_OFFSET = 1008;
const unsigned int SRR2_ALIGN_OUTPUT_UPDATED_COMPLETED_SIZE = 2;