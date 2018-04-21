#!/bin/bash

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can1 up type can bitrate 125000

#candump can0,763:FFFFFFFF can0,765:FFFFFFFF &

cansend can0 762#FF00000000000000
cansend can0 764#FF00000000000000


cansend can0 762#F103020000000000
cansend can0 764#F103020000000000


#candump can1,763:FFFFFFFF can1,765:FFFFFFFF

