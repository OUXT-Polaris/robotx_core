#include <ros/ros.h>

#include <dynamixel_driver.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dynamixel_driver_node");
  dynamixel_driver driver;
  ros::spin();
  return 0;
}