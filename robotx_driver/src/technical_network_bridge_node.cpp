/**
 * @mainpage heartbeat_publisher_node
 * ROS node for publish robotx heartbeat protocol in TCP/IP
 * @author Masaya Kataoka
 * @date 2018.06.09
 * @image html images/logo.jpg
 */

// headers in this package
#include <technical_network_bridge.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "heartbeat_publisher_node");
  technical_network_bridge bridge;
  ros::spin();
  return 0;
}
