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

#include "radar_interface/can_tools.h"

namespace can_tools {
CANParseInfo::CANParseInfo(uint8_t start_bit, uint8_t length, bool is_signed,
                           float scale, float offset, float min, float max) {
  div_t div_res;
  int start_byte_bit, start_byte_bit_invert;
  is_signed_ = is_signed;
  scale_ = scale;
  offset_ = offset;
  min_ = min;
  max_ = max;
  length_ = length;

  div_res = div(start_bit, 8);
  start_byte_ = div_res.quot;
  start_byte_bit = div_res.rem;
  start_byte_bit_invert = 7 - start_byte_bit;
  std::cout << (int)start_byte_ << "," << (int)start_byte_bit << ","
            << (int)start_byte_bit_invert << "," << std::endl;
  div_res = div((start_byte_bit_invert + 8 * start_byte_) + length - 1, 8);
  end_byte_ = div_res.quot;
  end_byte_shift_ = 7 - div_res.rem;
  std::cout << (int)end_byte_ << "," << (int)end_byte_shift_ << std::endl;

  sign_bit_mask_ = 1 << start_byte_bit;
  complement_offset_ = -2 * pow(2, length_ - 1);
  complement_max_positive_ = pow(2, length_ - 1) - 1;

  for (size_t i = 0; i < 8; i++) {
    mask_[i] = 0x00;

    if (i == start_byte_ && i == end_byte_) {
      for (size_t j = 0; j < length_; j++) {
        // std::cout << std::bitset<8>(mask_[i]) << ",11" << std::endl;
        mask_[i] = (mask_[i] << 1) | (0x01 << end_byte_shift_);
        std::cout << std::bitset<8>(mask_[i]) << ",1" << std::endl;
      };
    } else if (i == start_byte_ && i != end_byte_) {
      for (size_t j = 0; j <= start_byte_bit; j++) {
        mask_[i] = (mask_[i] << 1) | 0x01;
        std::cout << std::bitset<8>(mask_[i]) << ",2" << std::endl;
      }
    } else if (i > start_byte_ && i < end_byte_) {
      mask_[i] = 0xff;
      std::cout << std::bitset<8>(mask_[i]) << ",3" << std::endl;
    } else if (i > start_byte_ && i == end_byte_) {
      for (size_t j = 0; j < (8 - end_byte_shift_); j++) {
        mask_[i] = (mask_[i] << 1) | (0x01 << end_byte_shift_);
        std::cout << std::bitset<8>(mask_[i]) << ",4" << std::endl;
      }
    }
    std::cout << std::bitset<8>(mask_[i]) << ",";
  }
  std::cout << std::endl;
  std::cout << std::bitset<8>(sign_bit_mask_) << std::endl;
  std::cout << std::dec << start_byte_bit << "," << (int)start_byte_ << ","
            << (int)start_byte_bit << "," << (int)end_byte_ << ","
            << (int)end_byte_shift_ << "," << std::endl;
};
} // namespace can_tools