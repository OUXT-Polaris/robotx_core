#include <signal_lamp_driver.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_lamp_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  singal_lamp_driver driver(nh,pnh);
  driver.run();
  ros::spin();
  return 0;
}