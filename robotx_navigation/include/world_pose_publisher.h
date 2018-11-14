#ifndef WORLD_POSE_PUBLISHER_H_INCLUDED
#define WORLD_POSE_PUBLISHER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

//headers in this package
#include <UTM.h>

//headers in Boost
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

struct global_pose
{
    sensor_msgs::NavSatFix fix;
    geometry_msgs::QuaternionStamped true_course;
};

struct utm_position
{
    double x;
    double y;
};

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
    boost::shared_ptr<message_filters::Synchronizer<sync_policy> > sync_ptr_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix> > fix_sub_ptr_;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped> > twist_sub_ptr_;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::QuaternionStamped> > true_course_sub_ptr_;
    std::string fix_topic_;
    std::string twist_topic_;
    std::string true_course_topic_;
    tf2_ros::TransformBroadcaster broadcaster_;
    std::string world_frame_;
    boost::optional<global_pose> origin_;
    boost::optional<utm_position> origin_utm_;
    std::string world_pose_topic_;
    ros::Publisher world_pose_pub_;
    std::string world_odom_topic_;
    ros::Publisher world_odom_pub_;
    void gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix,
        const geometry_msgs::TwistStampedConstPtr& twist,
        const geometry_msgs::QuaternionStampedConstPtr true_course);
};
#endif  //WORLD_POSE_PUBLISHER_H_INCLUDED