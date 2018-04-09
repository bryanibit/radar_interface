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

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;
const unsigned int ESR_XCP_PAYLOAD_SIZE = 8568;
const unsigned int ESR_DSP_VER_1_OFFSET = 93;
const unsigned int ESR_DSP_VER_1_SIZE = 1;
const unsigned int ESR_DSP_VER_2_OFFSET = 100;
const unsigned int ESR_DSP_VER_2_SIZE = 1;
const unsigned int ESR_DSP_VER_3_OFFSET = 107;
const unsigned int ESR_DSP_VER_3_SIZE = 1;
const unsigned int ESR_SCAN_TYPE_OFFSET = 30;
const unsigned int ESR_SCAN_TYPE_SIZE = 1;
const unsigned int ESR_SCAN_INDEX_OFFSET = 14;
const unsigned int ESR_SCAN_INDEX_SIZE = 2;
const unsigned int ESR_TGT_RPT_CNT_OFFSET = 4350;
const unsigned int ESR_TGT_RPT_CNT_SIZE = 2;
const unsigned int ESR_TGT_RNG_OFFSET = 4626;
const unsigned int ESR_TGT_RNG_SIZE = 2;
const unsigned int ESR_TIME_STAMP_OFFSET = 83;
const unsigned int ESR_TIME_STAMP_SIZE = 4;
const unsigned int ESR_MAX_TARGET_NUM = 64;