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
#include "radar_interface/RadarTrack.h"
#include "radar_interface/RadarTrackArray.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/MarkerArray.h"
#include <math.h>
#include <tf/tf.h>
// #include "string.h"

class Colormap {
public:
  Colormap();
  virtual ~Colormap();
  void setColormap(const float min_color_value, const float max_color_value);
  std_msgs::ColorRGBA getColor(const float value);

private:
  // Colormap variables
  float min_color_value_;
  float max_color_value_;
  float saturation_;
  float intensity_;
  float start_color_;
  float end_color_;
  float value_interval_;
  float color_interval_;
};

class RadarMarkers {

public:
  RadarMarkers();
  virtual ~RadarMarkers();
  void setTrailLength(const int &length);
  void updateTrackMarkers(const radar_interface::RadarTrackArray &track_array);
  void
  updateTargetMarkers(const radar_interface::RadarTargetArray &target_array);
  void initializePublisher(ros::NodeHandle *node_handler);
  std_msgs::ColorRGBA getStatusColor(const int status);

  int trail_length_;
  bool trail_ON_;
  int marker_num_;
  visualization_msgs::MarkerArray marker_array_;
  visualization_msgs::MarkerArray speed_array_;
  visualization_msgs::MarkerArray accel_array_;
  visualization_msgs::MarkerArray strip_array_;
  std::string marker_namespace_;
  uint8_t marker_type_;
  ros::NodeHandle *nh_;
  ros::Publisher marker_pub_;
  ros::Publisher speed_pub_;
  ros::Publisher accel_pub_;
  ros::Publisher strip_pub_;
  Colormap colormap_;
};
