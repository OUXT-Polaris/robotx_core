#ifndef GPS_TO_BASE_LINK_H_INCLUDED
#define GPS_TO_BASE_LINK_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

//headers in this package
#include <robotx_driver/gps_to_base_linkConfig.h>

//headers in STL
#include <map>
#define _USE_MATH_DEFINES
#include<math.h>

class gps_to_base_link
{
public:
    gps_to_base_link(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~gps_to_base_link();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void configure_callback_(robotx_driver::gps_to_base_linkConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robotx_driver::gps_to_base_linkConfig> server_;
    dynamic_reconfigure::Server<robotx_driver::gps_to_base_linkConfig>::CallbackType callback_func_type_;
    tf2_ros::StaticTransformBroadcaster broadcaster_;
    geometry_msgs::Quaternion convert(double r,double p, double y);
};

#endif  //GPS_TO_BASE_LINK_H_INCLUDED