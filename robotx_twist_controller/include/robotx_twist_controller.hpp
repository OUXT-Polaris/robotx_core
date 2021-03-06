#ifndef ROBOTX_TWIST_CONTROLLER_HPP_INCLUDED
#define ROBOTX_TWIST_CONTROLLER_HPP_INCLUDED

// usual header for ROS
#include <ros/ros.h>

// messages to use
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>

// for std lib
#include <iostream>
#include <mutex>

namespace robotx {
  namespace controller {
    class Twist {
    public:
      Twist();
      ~Twist();
      void CtrlUpdate();

    private:
      void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
      void CurStateCallback(const geometry_msgs::Twist::ConstPtr &msg);

      // node handler
      ros::NodeHandle nh_;

      // subscriber
      ros::Subscriber cmd_vel_sub_;
      ros::Subscriber current_state_sub_;
      // publisher
      ros::Publisher motor_command_pub_;

      // tmp_messages
      geometry_msgs::Twist cmd_vel_;
      geometry_msgs::Twist current_state_;

      // subscribe message mutex
      std::mutex cmd_vel_mtx_;
      std::mutex current_state_mtx_;

      // parameters
      double tread_;
      double max_left_vel_;
      double max_right_vel_;
      double kp_vel_;
      double ki_vel_;  
      double kp_ang_vel_;
      double ki_ang_vel_;  
      double pub_rate_;
    };
  }
}

#endif
