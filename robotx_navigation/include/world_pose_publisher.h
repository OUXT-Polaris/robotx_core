#ifndef WORLD_POSE_PUBLISHER_H_INCLUDED
#define WORLD_POSE_PUBLISHER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

//headers in Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>

//headers in STL
#include <mutex>

typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::NavSatFix, geometry_msgs::TwistStamped, geometry_msgs::QuaternionStamped> sync_policy;

class world_pose_publisher
{
public:
    world_pose_publisher(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~world_pose_publisher();
    void run();
private:
    std::mutex mtx_;
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
    std::string world_pose_topic_;
    ros::Publisher world_pose_pub_;
    std::string world_odom_topic_;
    ros::Publisher world_odom_pub_;
    std::string imu_topic_;
    ros::Subscriber imu_sub_;
    double publish_rate_;
    bool data_recieved_;
    sensor_msgs::NavSatFix fix_;
    geometry_msgs::TwistStamped twist_;
    geometry_msgs::QuaternionStamped true_course_;
    std_msgs::Header twist_header_;
    boost::circular_buffer<sensor_msgs::Imu> imu_data_;
    void publish_world_frame_();
    void imu_callback_(const sensor_msgs::Imu::ConstPtr msg);
    void gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix,
        const geometry_msgs::TwistStampedConstPtr& twist,
        const geometry_msgs::QuaternionStampedConstPtr true_course);
    geometry_msgs::Pose2D convert_to_pose2d_(geometry_msgs::Pose pose3d);
    geometry_msgs::Pose convert_to_pose3d_(geometry_msgs::Pose2D pose2d);
};
#endif  //WORLD_POSE_PUBLISHER_H_INCLUDED