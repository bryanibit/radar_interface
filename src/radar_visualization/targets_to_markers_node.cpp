#include "radar_interface/RadarTargetArray.h"
#include "radar_interface/radar_visualization/radar_visualization.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

class SubsriberPublisher {
public:
  SubsriberPublisher();
  ros::Subscriber sub_;
  ros::NodeHandle n_;
  void
  targetsCallback(const radar_interface::RadarTargetArray::ConstPtr &targets_msg);

private:
  RadarMarkers markers_;
};

SubsriberPublisher::SubsriberPublisher() {
  markers_.initializePublisher(&n_);
}
void SubsriberPublisher::targetsCallback(
    const radar_interface::RadarTargetArray::ConstPtr &targets_msg) {
    markers_.updateTargetMarkers(*targets_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "targets_to_markers");

  SubsriberPublisher sp;
  sp.sub_ = sp.n_.subscribe("/radar_targets", 1000,
                            &SubsriberPublisher::targetsCallback, &sp);

  ros::spin();

  return 0;
}
