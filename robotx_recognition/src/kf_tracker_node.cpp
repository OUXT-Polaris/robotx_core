#include <kf_tracker.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_tracker_node");
  kf_tracker tracker;
  ros::spin();
  return 0;
}