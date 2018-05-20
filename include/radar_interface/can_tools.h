#ifndef CAN_TOOLS_H
#define CAN_TOOLS_H

#include "linux/can.h"
#include <socketcan_interface/interface.h>
#include <socketcan_interface/string.h>

namespace can_tools {
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
  bool is_signed = std::is_signed<T>::value;

  for (size_t i = parse_info.START_BYTE; i <= parse_info.END_BYTE; i++) {
    int_value = (int_value << 8) | (frame[i] & parse_info.MASK[i]);
  }
  &int_value = &int_value >> parse_info.SHIFT;

  &value = int_value * parse_info.SCALE;
  if (is_signed && &value > parse_info.MAX) {
    &value = &value + 2 * parse_info.MIN;
  }
}
} // namespace can_tools

#endif