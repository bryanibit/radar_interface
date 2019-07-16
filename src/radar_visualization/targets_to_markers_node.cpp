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

#include "radar_interface/RadarTargetArray.h"
#include "radar_interface/radar_visualization/radar_visualization.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#define SRR2_MIN_RCS -24
#define SRR2_MAX_RCS 40

class SubsriberPublisher {
public:
  SubsriberPublisher();
  ros::Subscriber sub_;
  ros::NodeHandle n_;
  void targetsCallback(
      const radar_interface::RadarTargetArray::ConstPtr &targets_msg);

private:
  RadarMarkers markers_;
};

SubsriberPublisher::SubsriberPublisher() {
  markers_.initializePublisher(&n_);
  markers_.colormap_.setColormap(SRR2_MIN_RCS, SRR2_MAX_RCS);
}
void SubsriberPublisher::targetsCallback(
    const radar_interface::RadarTargetArray::ConstPtr &targets_msg) {
  markers_.updateTargetMarkers(*targets_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "targets_to_markers");

  SubsriberPublisher sp;
  sp.sub_ = sp.n_.subscribe("radar_targets", 1000,
                            &SubsriberPublisher::targetsCallback, &sp);

  ros::spin();

  return 0;
}
