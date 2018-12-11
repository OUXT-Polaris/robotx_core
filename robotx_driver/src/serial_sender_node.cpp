#include <serial_sender.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_sender_node");
  serial_sender sender;
  sender.run();
  ros::spin();
  return 0;
}