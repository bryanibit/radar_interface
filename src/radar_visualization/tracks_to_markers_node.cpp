#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/radar_visualization/radar_visualization.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "string.h"


class SubsriberPublisher {
public:
  SubsriberPublisher();
  ros::Subscriber sub_;
  ros::NodeHandle n_;
  void
  tracksCallback(const radar_interface::RadarTrackArray::ConstPtr &tracks_msg);

private:
  RadarMarkers markers_;
};

SubsriberPublisher::SubsriberPublisher() {
  markers_.initializePublisher(&n_);
}
void SubsriberPublisher::tracksCallback(
    const radar_interface::RadarTrackArray::ConstPtr &tracks_msg) {
    markers_.updateTrackMarkers(*tracks_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tracks_to_markers");

  SubsriberPublisher sp;
  sp.sub_ = sp.n_.subscribe("/radar_tracks", 1000,
                            &SubsriberPublisher::tracksCallback, &sp);

// std::string ime =sp.sub_.getTopic();
ROS_INFO("Subscribed to: %s" ,sp.sub_.getTopic().c_str());

  ros::spin();

  return 0;
}
