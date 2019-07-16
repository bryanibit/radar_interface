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

#include "radar_interface/RadarTarget.h"
#include "radar_interface/RadarTargetArray.h"
#include "radar_interface/TCPFrame.h"
#include "radar_interface/VehicleInfo.h"
#include "radar_interface/radar_visualization/radar_visualization.h"
#include "radar_interface/tcp_interface.h"

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include <cstdio>

#include <sstream>

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;
const unsigned int ESR_XCP_PAYLOAD_SIZE = 8568;
const unsigned int ESR_TIME_STAMP_OFFSET = 83;
const unsigned int ESR_TIME_STAMP_SIZE = 4;
const unsigned int ESR_MAX_TARGET_NUM = 64;
const float ESR_TGT_MIN_RCS = -10;
const float ESR_TGT_MAX_RCS = 40;

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

const unsigned int ESR_TGT_RNG_OFFSET = 4358;
const unsigned int ESR_TGT_RNG_SIZE = 2;
const float ESR_TGT_RNG_SCALE = 128.0;
const unsigned int ESR_TGT_RNG_RATE_OFFSET = 4626;
const unsigned int ESR_TGT_RNG_RATE_SIZE = 2;
const float ESR_TGT_RNG_RATE_SCALE = 128.0;
const unsigned int ESR_TGT_AZIM_OFFSET = 4894;
const unsigned int ESR_TGT_AZIM_SIZE = 2;
const float ESR_TGT_AZIM_SCALE = 128.0;
const unsigned int ESR_TGT_RCS_OFFSET = 5028;
const unsigned int ESR_TGT_RCS_SIZE = 2;
const float ESR_TGT_RCS_SCALE = 128.0;

const unsigned int ESR_HOST_SPEED_OFFSET = 67;
const unsigned int ESR_HOST_SPEED_SIZE = 2;
const float ESR_HOST_SPEED_SCALE = 128.0;
const unsigned int ESR_HOST_YAW_RATE_OFFSET = 75;
const unsigned int ESR_HOST_YAW_RATE_SIZE = 2;
const float ESR_HOST_YAW_RATE_SCALE = 128.0;
