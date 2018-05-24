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
  Colormap colormap;
};
