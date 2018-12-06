// headers in this package
#include <frame_publisher.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "frame_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  frame_publisher publisher(nh,pnh);
  ros::spin();
  return 0;
}