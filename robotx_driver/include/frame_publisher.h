#ifndef FRAME_PUBLISHER_H_INCLUDED
#define FRAME_PUBLISHER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

//headers in this package
#include <robotx_driver/frame_publisherConfig.h>

//headers in STL
#include <map>
#define _USE_MATH_DEFINES
#include<math.h>

class frame_publisher
{
public:
    frame_publisher(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~frame_publisher();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void configure_callback_(robotx_driver::frame_publisherConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robotx_driver::frame_publisherConfig> server_;
    dynamic_reconfigure::Server<robotx_driver::frame_publisherConfig>::CallbackType callback_func_type_;
    tf2_ros::StaticTransformBroadcaster broadcaster_;
    geometry_msgs::Quaternion convert(double r,double p, double y);
};
#endif  //FRAME_PUBLISHER_H_INCLUDED