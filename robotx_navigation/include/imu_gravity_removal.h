#ifndef IMU_GRAVIRY_REMOVAL_H_INCLUDED
#define IMU_GRAVIRY_REMOVAL_H_INCLUDED

// headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistTim.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <iostream>
#include <chrono>


class imu_gravity_removal {
 public:
  struct parameters {
    double LPF_const_value;
    std::string input_imu_topic;
    parameters() {
      ros::param::param<double>("/LPF_const_value", LPF_const_value, 0.8);
      ros::param::param<std::string>(ros::this_node::getName() + "/input_imu_topic", input_imu_topic,
                                     ros::this_node::getName() + "/input_imu");
    }
  };

  imu_gravity_removal();

  ~imu_gravity_removal();

 private:

    void imu_CB_(const sensor_msgs::Imu msg);
    ros::NodeHandle nh_;

    const parameters params_;

    ros::Publisher rm_gravity_pub_;
    ros::Subscriber raw_imu_sub_;

    std::chrono::system_clock::time_point  start_time_, end_time_; // 型は auto で可
    double sec_;

    struct linear_speeds{
        double x,y,z;
    };
    linear_speeds grav_,vel_,old_acc_,old_raw_acc_;

};
#endif  // IMU_GRAVIRY_REMOVAL_H_INCLUDED