#ifndef SIMPLE_NAVIGATOR_H_INCLUDED
#define SIMPLE_NAVIGATOR_H_INCLUDED

//headers in this package
#include <robot_state.h>

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

//headers in ceres
#include <ceres/ceres.h>

struct cost_function
{
    template <typename T>
    bool operator()(
        const T* const radius,
        const T* const rot,
        T* residual
    ) const {
        residual[0] = 0;
    };
};

class simple_navigator
{
public:
    simple_navigator();
    ~simple_navigator();
private:
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> _sync_policy;
    ros::NodeHandle _nh;
    ros::Subscriber _euclidean_cluster_sub;
    void _euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg);
    ros::Subscriber _goal_pose_sub;
    void _goal_pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    message_filters::Subscriber<geometry_msgs::PoseStamped> _robot_pose_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> _twist_sub;
    message_filters::Synchronizer<_sync_policy> _pose_twist_sync;
    //ros::Subscriber _robot_pose_sub;
    //void _robot_pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    double _publish_rate;
    double _min_search_radius;
    double _max_search_radius;
    double _max_rotation_speed;
    double _max_speed;
    ceres::Problem _problem;
};
#endif  //SIMPLE_NAVIGATOR_H_INCLUDED