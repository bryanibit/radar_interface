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

#ifndef CAN_TOOLS_H
#define CAN_TOOLS_H

#include "linux/can.h"
#include <socketcan_interface/interface.h>
#include <socketcan_interface/string.h>
// #include <type_traits>
#include <bitset>
#include <limits>

namespace can_tools {

class CANParseInfo {
private:
  uint8_t start_bit_;
  uint8_t length_;
  float scale_;
  float offset_;
  float min_;
  float max_;
  int complement_offset_;
  int complement_max_positive_;
  unsigned char mask_[8];
  unsigned char sign_bit_mask_;
  uint8_t start_byte_;
  uint8_t end_byte_;
  uint8_t end_byte_shift_;
  bool is_signed_;
  int dummy;

public:
  CANParseInfo(uint8_t start_bit, uint8_t length, bool is_signed, float scale,
               float offset, float min, float max);

  template <typename T> void parseValue(const can::Frame &frame, T *value) {
    int int_value = 0;
    unsigned char temp, temp1, temp2, temp3, temp4, temp5;

    for (size_t i = start_byte_; i <= end_byte_; i++) {
      int_value = (int_value << 8) | (frame.data[i] & mask_[i]);
    }
    int_value = int_value >> end_byte_shift_;

    if (is_signed_ && (frame.data[start_byte_] & sign_bit_mask_) &&
        int_value > complement_max_positive_) {
      int_value = int_value + complement_offset_;
    }
    *value = int_value * scale_;
    *value = *value + offset_;
  }

  template <typename T> void setValue(can::Frame *frame, T &value) {
    // assumes that the frame slot is 0-filled
    T value_clamped = std::max(T(min_), std::min(value, T(max_)));
    int int_value = (int((value_clamped - offset_) / scale_))
                    << end_byte_shift_;
    unsigned char temp, temp1, temp2, temp3, temp4, temp5;
    for (size_t i = start_byte_; i <= end_byte_; i++) {
      temp = (int_value >> (8 * (end_byte_ - i))) & mask_[i];
      temp1 = frame->data[i] & (~mask_[i]);
      temp2 = frame->data[i];
      temp3 = (!mask_[i]);
      frame->data[i] = frame->data[i] & (~mask_[i]);
      temp4 = frame->data[i];
      temp5 = frame->data[i] | temp;
      frame->data[i] = frame->data[i] | temp;
    }
  }
};

struct CANParseValueInfo {
  unsigned short int MSG_ID;
  unsigned short int START_BYTE;
  unsigned short int END_BYTE;
  unsigned short int SHIFT;
  float SCALE;
  float OFFSET;
  float MIN;
  float MAX;
  unsigned char MASK[8];
  unsigned char SIGN_BIT_MASK;
};

template <typename T>
void setValue(can::Frame *frame, T &value,
              const CANParseValueInfo &parse_info) {
  // assumes that the frame slot is 0-filled
  T value_clamped =
      std::max(T(parse_info.MIN), std::min(value, T(parse_info.MAX)));
  int int_value = (int((value_clamped - parse_info.OFFSET) / parse_info.SCALE))
                  << parse_info.SHIFT;
  unsigned char temp, temp1, temp2, temp3, temp4, temp5;
  for (size_t i = parse_info.START_BYTE; i <= parse_info.END_BYTE; i++) {
    temp = (int_value >> (8 * (parse_info.END_BYTE - i))) & parse_info.MASK[i];
    temp1 = frame->data[i] & (~parse_info.MASK[i]);
    temp2 = frame->data[i];
    temp3 = (!parse_info.MASK[i]);
    frame->data[i] = frame->data[i] & (~parse_info.MASK[i]);
    temp4 = frame->data[i];
    temp5 = frame->data[i] | temp;
    frame->data[i] = frame->data[i] | temp;
  }
}

template <typename T>
void parseValue(const can::Frame &frame, T *value,
                const CANParseValueInfo &parse_info) {
  // assumes that the frame slot is 0-filled

  int int_value = 0;
  unsigned char temp, temp1, temp2, temp3, temp4, temp5;
  bool is_signed = std::numeric_limits<T>::is_signed;

  for (size_t i = parse_info.START_BYTE; i <= parse_info.END_BYTE; i++) {
    int_value = (int_value << 8) | (frame.data[i] & parse_info.MASK[i]);
  }
  int_value = int_value >> parse_info.SHIFT;

  *value = int_value * parse_info.SCALE;
  if (is_signed && *value > parse_info.MAX) {
    *value = *value + 2 * parse_info.MIN;
  }
}

} // namespace can_tools

#endif