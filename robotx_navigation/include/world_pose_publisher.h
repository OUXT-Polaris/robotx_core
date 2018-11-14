#ifndef WORLD_POSE_PUBLISHER_H_INCLUDED
#define WORLD_POSE_PUBLISHER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

//headers in this package
#include <UTM.h>

typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::NavSatFix, geometry_msgs::TwistStamped, geometry_msgs::QuaternionStamped> sync_policy;

class world_pose_publisher
{
public:
    world_pose_publisher(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~world_pose_publisher();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix,
        const geometry_msgs::TwistStampedConstPtr& twist,
        const geometry_msgs::QuaternionStampedConstPtr quat);
};
#endif  //WORLD_POSE_PUBLISHER_H_INCLUDED