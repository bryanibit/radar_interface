#include "radar_interface/radar_visualization.h"

RadarMarkers::RadarMarkers() {
  trail_length_ = 0;
  trail_ON_ = false;
  marker_num_ = 64;
  marker_namespace_="radar";
  marker_type_=visualization_msgs::Marker::CUBE;
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

void RadarMarkers::updateMarkers(
    const radar_interface::RadarTrackArray &track_array) {
  for (size_t i = 0; i < marker_num_; i++) {
    if (track_array.tracks[i].status > 0) {
      marker_array_.markers[i].id=track_array.tracks[i].id;
      marker_array_.markers[i].pose.position.x=track_array.tracks[i].pos.x;
      marker_array_.markers[i].pose.position.y=track_array.tracks[i].pos.y;
      marker_array_.markers[i].action=visualization_msgs::Marker::ADD;
      marker_array_.markers[i].type=visualization_msgs::Marker::CUBE;
    } else {
      marker_array_.markers[i].action=visualization_msgs::Marker::DELETE;
    }
  }
}