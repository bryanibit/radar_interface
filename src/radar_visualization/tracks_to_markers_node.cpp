#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/radar_visualization/radar_visualization.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

class SubsriberPublisher {
public:
  SubsriberPublisher();
  ros::Subscriber sub_;
  ros::Publisher pub_;
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
  // sp.sub_ = sp.n_.subscribe("/radar_tracks", 1000,
  //                           &SubsriberPublisher::tracksCallback, &sp);
    sp.sub_ = sp.n_.subscribe("/custom_chatter", 1000,
                            &SubsriberPublisher::tracksCallback, &sp);
  sp.pub_ =
      sp.n_.advertise<visualization_msgs::MarkerArray>("/track_markers", 1000);

  ros::spin();

  return 0;
}
