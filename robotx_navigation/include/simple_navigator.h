#ifndef SIMPLE_NAVIGATOR_H_INCLUDED
#define SIMPLE_NAVIGATOR_H_INCLUDED

//headers in robotx_debag_tools
#include <robotx_debag_tools/robotx_glog_tools.h>

//headers in this package
#include <robot_state.h>
#include <euclidean_cluster_buffer.h>

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

//headers in ceres
#include <ceres/ceres.h>

//headers in boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class simple_navigator
{
public:
    simple_navigator();
    ~simple_navigator();
private:
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> _sync_policy;
    ros::NodeHandle _nh;
    ros::Publisher _marker_pub;
    ros::Publisher _twist_cmd_pub;
    ros::Subscriber _euclidean_cluster_sub;
    void _euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg);
    ros::Subscriber _goal_pose_sub;
    void _goal_pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    message_filters::Subscriber<geometry_msgs::PoseStamped> _robot_pose_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> _twist_sub;
    message_filters::Synchronizer<_sync_policy> _pose_twist_sync;
    void _pose_callback(const geometry_msgs::PoseStampedConstPtr robot_pose_msg,const geometry_msgs::TwistStampedConstPtr twist_msg);
    void _publish_twist_cmd();
    void _publish_marker(double search_radius);
    //double _get_distance(double r, double theta, );
    robot_state _robot_state;
    double _publish_rate;
    double _max_search_radius;
    double _max_rotation_speed;
    double _max_speed;
    double _target_duration;
    double _min_cluster_length;
    double _max_cluster_length;
    double _buffer_length;
    double _get_total_distance();
    std::string _map_frame;
    boost::shared_ptr<euclidean_cluster_buffer> _buffer;
    double _get_search_radius(robot_state_info state_info);
    // debagger
    robotx_glog_tool debag_tools;
    // ceres params
    ceres::Problem _problem;
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
};
#endif  //SIMPLE_NAVIGATOR_H_INCLUDED