#include <sc30_driver.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sc30_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  sc30_driver driver(nh,pnh);
  ros::spin();
  return 0;
}