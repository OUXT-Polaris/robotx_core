#ifndef IMU_GRAVIRY_REMOVAL_H_INCLUDED
#define IMU_GRAVIRY_REMOVAL_H_INCLUDED

// headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <iostream>
#include <chrono>
#include <tf/transform_broadcaster.h>


class imu_gravity_removal {
 public:
  struct parameters {
    double LPF_const_value;
    std::string frame_id,input_imu_topic,nmea_true_cource_topic;
    double acc_x_offset;
    double acc_y_offset;
    double acc_z_offset;
    double gyro_x_offset;
    double gyro_y_offset;
    double gyro_z_offset;
    parameters() {
      ros::param::param<double>("/LPF_const_value", LPF_const_value, 0.8);
      ros::param::param<double>(ros::this_node::getName() + "/acc_x_offset", acc_x_offset,0.0);
      ros::param::param<double>(ros::this_node::getName() + "/acc_y_offset", acc_y_offset,0.0);
      ros::param::param<double>(ros::this_node::getName() + "/acc_z_offset", acc_z_offset,0.0);
      ros::param::param<double>(ros::this_node::getName() + "/gyro_x_offset", gyro_x_offset,0.0);
      ros::param::param<double>(ros::this_node::getName() + "/gyro_y_offset", gyro_y_offset,0.0);
      ros::param::param<double>(ros::this_node::getName() + "/gyro_z_offset", gyro_z_offset,0.0);
      ros::param::param<std::string>(ros::this_node::getName() + "/nmea_true_cource_topic_name", nmea_true_cource_topic,"/sc30_driver_node/true_course");
      ros::param::param<std::string>(ros::this_node::getName() + "/frame_id", frame_id,"/imu");
      ros::param::param<std::string>(ros::this_node::getName() + "/input_imu_topic", input_imu_topic,
                                     ros::this_node::getName() + "/input_imu");
    }
  };

  imu_gravity_removal();

  ~imu_gravity_removal();

 private:

    void imu_CB_(const sensor_msgs::Imu msg);
    void pose_CB_(const geometry_msgs::QuaternionStamped msg);
    ros::NodeHandle nh_;

    const parameters params_;

    ros::Publisher rm_gravity_pub_;
    ros::Publisher rm_gravity_pub_noStamped_;
    ros::Subscriber raw_imu_sub_;
    ros::Subscriber pose_sub_;

    geometry_msgs::Quaternion pose_;

    std::chrono::system_clock::time_point  start_time_, end_time_; // 型は auto で可
    double sec_;

    struct linear_speeds{
        double x,y,z;
    };
    linear_speeds grav_,vel_,old_acc_,old_raw_acc_;

};
#endif  // IMU_GRAVIRY_REMOVAL_H_INCLUDED