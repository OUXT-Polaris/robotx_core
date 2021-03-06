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
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <sensor_msgs/Imu.h>

//headers in Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/optional.hpp>

//headers in STL
#include <mutex>

/* typedef message_filters::sync_policies::ApproximateTime */
    /* <sensor_msgs::NavSatFix, geometry_msgs::TwistStamped, geometry_msgs::QuaternionStamped> sync_policy; */

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
    /*
    boost::shared_ptr<message_filters::Synchronizer<sync_policy> > sync_ptr_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix> > fix_sub_ptr_;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped> > twist_sub_ptr_;
    */
    ros::Subscriber fix_sub;
    ros::Subscriber twist_sub;
    ros::Subscriber true_course_sub;
    void fix_callback_(sensor_msgs::NavSatFix msg);
    void twist_callback_(geometry_msgs::TwistStamped msg);
    void true_course_callback_(geometry_msgs::QuaternionStamped msg);

    std::string fix_topic_;
    std::string gps_twist_topic_;
    std::string true_course_topic_;
    tf2_ros::TransformBroadcaster broadcaster_;
    std::string world_frame_;
    std::string robot_frame_;
    std::string world_pose_topic_;
    ros::Publisher world_pose_pub_;
    std::string world_odom_topic_;
    ros::Publisher world_odom_pub_;
    std::string imu_topic_;
    ros::Subscriber imu_sub_;
    ros::Publisher twist_pub_;
    double publish_rate_;
    bool data_recieved_;
    sensor_msgs::NavSatFix fix_;
    geometry_msgs::TwistStamped twist_;
    geometry_msgs::QuaternionStamped true_course_;
    std_msgs::Header twist_header_;
    void publish_world_frame_();
    /*
    void gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix,
        const geometry_msgs::TwistStampedConstPtr& twist,
        const geometry_msgs::QuaternionStampedConstPtr true_course);
        */
    void gnss_callback_();
    void imu_callback_(const sensor_msgs::Imu::ConstPtr msg);
    volatile bool imu_reset_flag_;
    double x_trans_imu_;
    double y_trans_imu_;
    double theta_trans_imu_;
    boost::optional<ros::Time> last_imu_timestamp_;
    void get_rpy_(const geometry_msgs::Quaternion &q, double &roll,double &pitch,double &yaw);
    void get_quat_(double roll,double pitch,double yaw,geometry_msgs::Quaternion &q);
    double yawrate_;
    double dv_;
    double v_;
};
#endif  //WORLD_POSE_PUBLISHER_H_INCLUDED
