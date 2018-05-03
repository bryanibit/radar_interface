#include "radar_interface/radar_visualization.h"

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
  for (size_t i = 0; i < marker_num_; i++) {
    if (track_array.tracks[i].status > 0) {
      marker_array_.markers[i].id = track_array.tracks[i].id;
      marker_array_.markers[i].pose.position.x = track_array.tracks[i].pos.x;
      marker_array_.markers[i].pose.position.y = track_array.tracks[i].pos.y;
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
    } else {
      marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  marker_pub_.publish(marker_array_);
}
void RadarMarkers::updateTargetMarkers(
    const radar_interface::RadarTargetArray &target_array) {
  for (size_t i = 0; i < marker_num_; i++) {
    if (target_array.targets[i].status != 0) {
      marker_array_.markers[i].id = i;
      marker_array_.markers[i].pose.position.x =
          target_array.targets[i].range * cos(target_array.targets[i].azimuth);
      marker_array_.markers[i].pose.position.y =
          target_array.targets[i].range * sin(target_array.targets[i].azimuth);
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
      marker_array_.markers[i].color =
          colormap.getColor(target_array.targets[i].rcs);
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
  marker_pub_ =
      nh_->advertise<visualization_msgs::MarkerArray>("markers", 1000);
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