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

#include "radar_interface/RadarTrackArray.h"
#include "radar_interface/radar_visualization/radar_visualization.h"
#include "ros/ros.h"
#include "string.h"
#include "visualization_msgs/MarkerArray.h"

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

SubsriberPublisher::SubsriberPublisher() { markers_.initializePublisher(&n_); }
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
  ROS_INFO("Subscribed to: %s", sp.sub_.getTopic().c_str());

  ros::spin();

  return 0;
}
