cmake_minimum_required(VERSION 2.8.3)
project(radar_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS genmsg message_generation roscpp std_msgs visualization_msgs socketcan_interface socketcan_bridge)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
  srr2/AlignmentInfoSRR2.msg
  VehicleInfo.msg
  TCPFrame.msg
  Vector2.msg
  RadarTrackArray.msg
  RadarTrack.msg
  RadarTarget.msg
  RadarTargetArray.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  message_runtime
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/radar_interface.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/radar_interface_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(tcp_interface
  include/radar_interface/tcp_interface.h
  src/tcp_interface.cpp
)

add_dependencies(tcp_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tcp_interface
  ${catkin_LIBRARIES}
)

add_library(radar_visualization
  include/radar_interface/radar_visualization.h
  src/radar_visualization.cpp
)

add_dependencies(radar_visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_visualization
  ${catkin_LIBRARIES}
)

add_executable(esr_targets_eth
  include/radar_interface/delphi_esr/esr_targets_eth.h
  src/delphi_esr/esr_targets_eth_node.cpp
)
add_dependencies(esr_targets_eth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(esr_targets_eth
  ${catkin_LIBRARIES}
  tcp_interface
  radar_visualization
)

add_library(esr_tracks_can
  include/radar_interface/delphi_esr/esr_tracks_can.h
  src/delphi_esr/esr_tracks_can.cpp
)

add_executable(esr_tracks_can_node
  include/radar_interface/delphi_esr/esr_tracks_can_node.h
  src/delphi_esr/esr_tracks_can_node.cpp
)
add_dependencies(esr_tracks_can_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(esr_tracks_can_node
  ${catkin_LIBRARIES}
  radar_visualization
  esr_tracks_can
)

add_executable(srr2_tracks_eth
  include/radar_interface/delphi_srr2/srr2_tracks_eth.h
  src/delphi_srr2/srr2_tracks_eth_node.cpp
)
add_dependencies(srr2_tracks_eth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(srr2_tracks_eth
  ${catkin_LIBRARIES}
  tcp_interface
  radar_visualization
)

add_executable(vehicle_to_srr2_node
  src/delphi_srr2/vehicle_to_srr2_node.cpp
)
add_dependencies(vehicle_to_srr2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_to_srr2_node
  ${catkin_LIBRARIES}
  vehicle_to_srr2
)

add_library(vehicle_to_srr2
  include/radar_interface/delphi_srr2/vehicle_to_srr2.h
  src/delphi_srr2/vehicle_to_srr2.cpp
)
add_dependencies(vehicle_to_srr2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_to_srr2
  ${catkin_LIBRARIES}
  can_tools
)

add_executable(vehicle_to_esr_node
  src/delphi_esr/vehicle_to_esr_node.cpp
)
add_dependencies(vehicle_to_esr_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_to_esr_node
  ${catkin_LIBRARIES}
  vehicle_to_esr
)

add_library(vehicle_to_esr
  include/radar_interface/delphi_esr/vehicle_to_esr.h
  src/delphi_esr/vehicle_to_esr.cpp
)
add_dependencies(vehicle_to_esr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_to_esr
  ${catkin_LIBRARIES}
  can_tools
)

add_library(can_tools
  src/can_tools.cpp
  include/radar_interface/can_tools.h
)
add_dependencies(can_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_tools
  ${catkin_LIBRARIES}
)

add_executable(imu_to_vehicle_node
  include/radar_interface/vehicle/imu_to_vehicle_node.h
  src/vehicle/imu_to_vehicle_node.cpp
)
add_dependencies(imu_to_vehicle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_to_vehicle_node
  ${catkin_LIBRARIES}
)
