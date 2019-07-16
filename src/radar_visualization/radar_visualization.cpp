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

#include "radar_interface/radar_visualization/radar_visualization.h"

RadarMarkers::RadarMarkers() {
  trail_length_ = 0;
  trail_ON_ = false;
  marker_num_ = 64;
  marker_namespace_ = "radar";
  marker_type_ = visualization_msgs::Marker::CUBE;
}

RadarMarkers::~RadarMarkers() {}

void RadarMarkers::setTrailLength(const int &length) {
  if (length > 0) {
    trail_ON_ = true;
    trail_length_ = length;
  } else {
    trail_ON_ = false;
  }
}

void RadarMarkers::updateTrackMarkers(
    const radar_interface::RadarTrackArray &track_array) {

  std::string frame_string = "radar";
  if (!track_array.header.frame_id.empty()) {
    frame_string = track_array.header.frame_id;
  }
  int marker_num = track_array.tracks.size();
  for (size_t i = 0; i < marker_num; i++) {
    marker_array_.markers[i].header.frame_id = frame_string;
    speed_array_.markers[i].header.frame_id = frame_string;
    accel_array_.markers[i].header.frame_id = frame_string;
    marker_array_.markers[i].id = track_array.tracks[i].id;
    speed_array_.markers[i].id = track_array.tracks[i].id;
    accel_array_.markers[i].id = track_array.tracks[i].id;
    // marker_array_.markers[i].lifetime=ros::Duration(0.1);
    // speed_array_.markers[i].lifetime=ros::Duration(0.1);
    // accel_array_.markers[i].lifetime=ros::Duration(0.1);

    // ROS_INFO("%s",marker_array_.markers[i].header.frame_id.c_str());
    if (track_array.tracks[i].status > 0) {
      // setting current position marker
      float track_orientation =
          atan2(track_array.tracks[i].pos.y, track_array.tracks[i].pos.x);
      tf::Quaternion q_pos =
          tf::createQuaternionFromRPY(0, 0, track_orientation);

      marker_array_.markers[i].pose.position.x = track_array.tracks[i].pos.x;
      marker_array_.markers[i].pose.position.y = track_array.tracks[i].pos.y;
      marker_array_.markers[i].pose.orientation.x = q_pos.getX();
      marker_array_.markers[i].pose.orientation.y = q_pos.getY();
      marker_array_.markers[i].pose.orientation.z = q_pos.getZ();
      marker_array_.markers[i].pose.orientation.w = q_pos.getW();
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array_.markers[i].type = marker_type_;
      marker_array_.markers[i].scale.x = 1;
      marker_array_.markers[i].scale.y =
          std::max(track_array.tracks[i].width, 0.25);
      marker_array_.markers[i].scale.z = 1;
      marker_array_.markers[i].color =
          getStatusColor(track_array.tracks[i].status);

      // setting current speed arrow
      float speed_length = sqrt(pow(track_array.tracks[i].vel.x, 2) +
                                pow(track_array.tracks[i].vel.y, 2));

      float speed_direction =
          atan2(track_array.tracks[i].vel.y, track_array.tracks[i].vel.x);

      tf::Quaternion q_speed =
          tf::createQuaternionFromRPY(0, 0, speed_direction);
      speed_array_.markers[i].pose.position =
          marker_array_.markers[i].pose.position;
      speed_array_.markers[i].pose.position.z = 0.5;
      speed_array_.markers[i].pose.orientation.x = q_speed.getX();
      speed_array_.markers[i].pose.orientation.y = q_speed.getY();
      speed_array_.markers[i].pose.orientation.z = q_speed.getZ();
      speed_array_.markers[i].pose.orientation.w = q_speed.getW();
      speed_array_.markers[i].scale.x = speed_length;
      speed_array_.markers[i].scale.y = 0.1;
      speed_array_.markers[i].scale.z = 0.1;
      speed_array_.markers[i].action = visualization_msgs::Marker::ADD;
      speed_array_.markers[i].type = visualization_msgs::Marker::ARROW;
      speed_array_.markers[i].color.r = 1;
      speed_array_.markers[i].color.g = 0;
      speed_array_.markers[i].color.b = 0;
      speed_array_.markers[i].color.a = 1;
      if (speed_array_.markers[i].scale.x == 0.0) {
        speed_array_.markers[i].action = visualization_msgs::Marker::DELETE;
      }

      // setting current acceleration arrow
      float accel_length = sqrt(pow(track_array.tracks[i].acc.x, 2) +
                                pow(track_array.tracks[i].acc.y, 2));

      float accel_direction =
          atan2(track_array.tracks[i].acc.y, track_array.tracks[i].acc.x);

      tf::Quaternion q_accel =
          tf::createQuaternionFromRPY(0, 0, accel_direction);
      accel_array_.markers[i].pose.position =
          marker_array_.markers[i].pose.position;
      accel_array_.markers[i].pose.position.z = 0.5;
      accel_array_.markers[i].pose.orientation.x = q_accel.getX();
      accel_array_.markers[i].pose.orientation.y = q_accel.getY();
      accel_array_.markers[i].pose.orientation.z = q_accel.getZ();
      accel_array_.markers[i].pose.orientation.w = q_accel.getW();
      accel_array_.markers[i].scale.x = accel_length;
      accel_array_.markers[i].scale.y = 0.1;
      accel_array_.markers[i].scale.z = 0.1;
      accel_array_.markers[i].action = visualization_msgs::Marker::ADD;
      accel_array_.markers[i].type = visualization_msgs::Marker::ARROW;
      accel_array_.markers[i].color.r = 0;
      accel_array_.markers[i].color.g = 0;
      accel_array_.markers[i].color.b = 1;
      accel_array_.markers[i].color.a = 1;

      if (accel_array_.markers[i].scale.x == 0.0) {
        accel_array_.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    } else {
      marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
      speed_array_.markers[i].action = visualization_msgs::Marker::DELETE;
      accel_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  marker_pub_.publish(marker_array_);
  speed_pub_.publish(speed_array_);
  accel_pub_.publish(accel_array_);
}
void RadarMarkers::updateTargetMarkers(
    const radar_interface::RadarTargetArray &target_array) {

  std::string frame_string = "radar";
  if (!target_array.header.frame_id.empty()) {
    frame_string = target_array.header.frame_id;
  }

  for (size_t i = 0; i < marker_num_; i++) {
    if (target_array.targets[i].status > -1) {

      marker_array_.markers[i].header.frame_id = frame_string;
      marker_array_.markers[i].id = i;
      marker_array_.markers[i].pose.position.x =
          target_array.targets[i].range * cos(target_array.targets[i].azimuth);
      marker_array_.markers[i].pose.position.y =
          target_array.targets[i].range * sin(target_array.targets[i].azimuth);
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;

      marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
      marker_array_.markers[i].color =
          colormap_.getColor(target_array.targets[i].rcs);
      if (target_array.targets[i].status == -1) {
        marker_array_.markers[i].type = visualization_msgs::Marker::SPHERE;
      }
    } else {
      marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  marker_pub_.publish(marker_array_);
}

void RadarMarkers::initializePublisher(ros::NodeHandle *node_handler) {
  nh_ = node_handler;
  marker_array_.markers.resize(marker_num_);
  speed_array_.markers.resize(marker_num_);
  accel_array_.markers.resize(marker_num_);
  for (size_t i = 0; i < marker_num_; i++) {
    marker_array_.markers[i].scale.x = 1;
    marker_array_.markers[i].scale.y = 1;
    marker_array_.markers[i].scale.z = 1;
    marker_array_.markers[i].color.r = 1;
    marker_array_.markers[i].color.a = 1;
  }

  marker_pub_ =
      nh_->advertise<visualization_msgs::MarkerArray>("markers_pos", 1000);
  speed_pub_ =
      nh_->advertise<visualization_msgs::MarkerArray>("markers_speed", 1000);
  accel_pub_ =
      nh_->advertise<visualization_msgs::MarkerArray>("markers_accel", 1000);
}

std_msgs::ColorRGBA RadarMarkers::getStatusColor(const int status) {

  std_msgs::ColorRGBA color;

  switch (status) {
  case 1: // yellow - new target
    color.r = 1;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    break;
  case 2: // orange - new updated target
    color.r = 1;
    color.g = 0.5;
    color.b = 0;
    color.a = 1;
    break;
  case 3: // green - updated
    color.r = 0;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    break;
  case 4: // black -  coasted
    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    break;
  case 5: // blue - merged
    color.r = 0;
    color.g = 0;
    color.b = 1;
    color.a = 1;
    break;
  case 6: // red - invalid coasted
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    break;
  case 7: // gray - new coasted
    color.r = 0.3;
    color.g = 0.3;
    color.b = 0.3;
    color.a = 1;
    break;
  }
  return color;
}

Colormap::Colormap() {}

Colormap::~Colormap() {}

void Colormap::setColormap(const float min_color_value,
                           const float max_color_value) {
  min_color_value_ = min_color_value;
  max_color_value_ = max_color_value;
  saturation_ = 1.0;
  intensity_ = 1.0;
  start_color_ = 240;
  end_color_ = 0;
  value_interval_ = max_color_value_ - min_color_value_;
  color_interval_ = end_color_ - start_color_;
};

std_msgs::ColorRGBA Colormap::getColor(const float value) {
  std_msgs::ColorRGBA color;
  color.a = intensity_;
  float hue =
      ((value - min_color_value_) / (value_interval_)) * color_interval_ +
      start_color_;

  if (saturation_ == 0) {
    color.r = intensity_;
    color.g = intensity_;
    color.b = intensity_;
  } else {
    int i;
    double f, p, q, t;

    if (hue == 360)
      hue = 0;
    else
      hue = hue / 60;

    i = (int)trunc(hue);
    f = hue - i;

    p = intensity_ * (1.0 - saturation_);
    q = intensity_ * (1.0 - (saturation_ * f));
    t = intensity_ * (1.0 - (saturation_ * (1.0 - f)));

    switch (i) {
    case 0:
      color.r = saturation_;
      color.g = t;
      color.b = p;
      break;

    case 1:
      color.r = q;
      color.g = saturation_;
      color.b = p;
      break;

    case 2:
      color.r = p;
      color.g = saturation_;
      color.b = t;
      break;

    case 3:
      color.r = p;
      color.g = q;
      color.b = saturation_;
      break;

    case 4:
      color.r = t;
      color.g = p;
      color.b = saturation_;
      break;

    default:
      color.r = saturation_;
      color.g = p;
      color.b = q;
      break;
    }
  }
  return color;
}