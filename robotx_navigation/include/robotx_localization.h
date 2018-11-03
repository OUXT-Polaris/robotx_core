#ifndef ROBOTX_LOCALIZATION_H_INCLUDED
#define ROBOTX_LOCALIZATION_H_INCLUDED

// headers in this package
#include <particle_filter.h>

// headers in ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

// headers in Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>

// headers in STL
#include <mutex>

// headers in Eigtn
#include <Eigen/Core>

class robotx_localization {
 public:
  struct parameters {
    std::string robot_frame;
    std::string publish_frame;
    std::string twist_topic;
    std::string fix_topic;
    std::string imu_topic;
    std::string reset_topic;
    int num_particles;
    int publish_rate;
    double ess_threshold;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/robot_frame", robot_frame,
                                     "base_footprint");
      ros::param::param<std::string>(ros::this_node::getName() + "/publish_frame", publish_frame, "map");
      ros::param::param<std::string>(ros::this_node::getName() + "/twist_topic", twist_topic,
                                     ros::this_node::getName() + "/twist");
      ros::param::param<std::string>(ros::this_node::getName() + "/reset_topic", reset_topic,"/reset");
      ros::param::param<std::string>(ros::this_node::getName() + "/fix_topic", fix_topic,
                                     ros::this_node::getName() + "/fix");
      ros::param::param<std::string>(ros::this_node::getName() + "/imu_topic", imu_topic,
                                     ros::this_node::getName() + "/imu");
      ros::param::param<int>(ros::this_node::getName() + "/num_particles", num_particles, 1000);
      ros::param::param<int>(ros::this_node::getName() + "/publish_rate", publish_rate, 100);
      ros::param::param<double>(ros::this_node::getName() + "/ess_threshold", ess_threshold,
                                (double)num_particles / (double)3);
    }
  };
  robotx_localization();
  ~robotx_localization();

 private:
  void initialize_particle_filter_();
  const parameters params_;
  void imu_callback_(sensor_msgs::Imu msg);
  void fix_callback_(sensor_msgs::NavSatFix msg);
  void twist_callback_(geometry_msgs::Twist msg);
  void reset_callback_(std_msgs::Empty msg);
  void update_frame_();
  bool is_sensor_ready_();
  boost::thread thread_update_frame_;
  ros::Subscriber fix_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber reset_sub_;
  ros::Publisher init_fix_pub_;
  ros::Publisher robot_pose_pub_;
  ros::Publisher odom_pub_;
  ros::NodeHandle nh_;
  sensor_msgs::NavSatFix last_fix_message_;
  sensor_msgs::NavSatFix init_measurement_;
  geometry_msgs::Twist last_twist_message_;
  volatile bool fix_recieved_;
  volatile bool twist_received_;
  volatile bool imu_recieved_;
  particle_filter *pfilter_ptr_;
  tf2_ros::TransformBroadcaster broadcaster_;
  std::mutex fix_mutex_;
  std::mutex twist_mutex_;
  std::mutex imu_mutex_;
  double init_yaw_;
  double yaw_;
};
#endif  // ROBOTX_LOCALIZATION_H_INCLUDED