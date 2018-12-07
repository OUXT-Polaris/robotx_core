// headers in ros
#include <hu_moments_matcher.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "marker_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  hu_moments_matcher(nh,pnh);
  ros::spin();
  return 0;
}