#include "radar_interface/RadarTrackArray.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "radar_interface/RadarTrack.h"

class RadarMarkers {

public:
  RadarMarkers();
  virtual ~RadarMarkers();
  void setTrailLength(const int &length);
  void updateMarkers(const radar_interface::RadarTrackArray &track_array);
  void initializePublisher(ros::NodeHandle* node_handler);

  int trail_length_;
  bool trail_ON_;
  int marker_num_;
  visualization_msgs::MarkerArray marker_array_;
  std::string marker_namespace_;
  uint8_t marker_type_;
  ros::NodeHandle* nh_;
  ros::Publisher marker_pub_;
  
};