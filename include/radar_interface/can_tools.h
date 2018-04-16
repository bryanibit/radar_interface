void setFrameValue(char *can_frame[8], const unsigned int bit_length) {
  uint8_t byte_start = bit_start / 8;
  uint8_t bit_end = bit_start + bit_length - 1;
  uint8_t byte_end = bit_end / 8;
  uint8_t byte_length=byte_end-byte_start+1;
  unsigned char mask[byte_length];
  
}