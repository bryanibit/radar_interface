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
void setValueInFrame(can::Frame *frame, T &value,
                     const CANParseValueInfo &parseInfo) {
  // assumes that the frame slot is 0-filled
  T value_clamped=std::max( T(parseInfo.MIN), std::min(value, T(parseInfo.MAX) ));
  int int_value = (int((value_clamped - parseInfo.OFFSET) / parseInfo.SCALE))
                  << parseInfo.SHIFT;
  unsigned char temp,temp1,temp2,temp3,temp4,temp5;
  for (size_t i = parseInfo.START_BYTE; i <= parseInfo.END_BYTE; i++) {
    temp = (int_value >> (8 * (parseInfo.END_BYTE - i))) &
           parseInfo.MASK[i];
    temp1=        frame->data[i] & (~parseInfo.MASK[i]); 
    temp2= frame->data[i];
    temp3= (!parseInfo.MASK[i]);
    frame->data[i] = frame->data[i] & (~parseInfo.MASK[i]);  
    temp4=    frame->data[i];
    temp5= frame->data[i] | temp;
    frame->data[i] = frame->data[i] | temp;
     
  }
}

}


#endif