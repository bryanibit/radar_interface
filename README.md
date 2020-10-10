# Origin of the project

The project is forked from [bitbucket](https://bitbucket.org/unizg-fer-lamor/radar_interface.git).

# How to setup Kvaser on Linux

## Method2

1. Install can-ultils(for socketcan):`sudo apt-get install can-utils`
2. Connect Kvaser to PC
3. Enable CAN
```sh
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```
4. Your Kvaser can port is enable with name `can0`, run `ifconfig -a` and can0 should show up. Connect Kvaser to another CAN device (esr) that is sending out data, then on terminal enter `candump can0`. You can find data with format like *can0 5E4 [8] 03 04 0A 0D 09 08 06 AA*.

# Read and write (speed) esr via Kvaser

After setting Kvaser can, the Kvaser **CAN** light should be turned on. And then install `socketcan` on ROS:
```sh
sudo apt-get install ros-kinetic-socketcan-bridge
sudo apt-get install ros-kinetic-socketcan-interface
```
And build the ROS package and run `roslaunch radar_interface delphi_esr_can.launch` to access the data throught socketcan from esr.
