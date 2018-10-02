#include <ros/console.h>
#include <robotx_twist_controller.hpp>

#include <cmath>

namespace rc = robotx::controller;
using namespace std;

rc::Twist::Twist()
  : tread_(3.0), max_left_vel_(5.0), max_right_vel_(5.0),
    kp_vel_(0.1), ki_vel_(0.1),  kp_ang_vel_(0.1), ki_ang_vel_(0.1),
    pub_rate_(25.0) {
  nh_.param<double>(ros::this_node::getName() + "/tread", tread_,
                    tread_);
  nh_.param<double>(ros::this_node::getName() + "/kp_vel", kp_vel_,
                    kp_vel_);
  nh_.param<double>(ros::this_node::getName() + "/ki_vel", ki_vel_,
                    ki_vel_);
  nh_.param<double>(ros::this_node::getName() + "/kp_ang_vel", kp_ang_vel_,
                    kp_ang_vel_);
  nh_.param<double>(ros::this_node::getName() + "/ki_ang_vel", ki_ang_vel_,
                    ki_ang_vel_);
  nh_.param<double>(ros::this_node::getName() + "/publish_rate", pub_rate_);
  motor_command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/wam_v/motor_command", 1);

  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &rc::Twist::CmdVelCallback, this);
  current_state_sub_ = nh_.subscribe("/vel", 1,
                                     &rc::Twist::CurStateCallback, this);

  // publish to stop
  std_msgs::Float64MultiArray cmd_msg;
  cmd_msg.data.push_back(0);
  cmd_msg.data.push_back(0);
  cmd_msg.data.push_back(0);
  cmd_msg.data.push_back(0);
  motor_command_pub_.publish(cmd_msg);
  ROS_INFO("Converter Ready....");
}

rc::Twist::~Twist() {}

void rc::Twist::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  cmd_vel_mtx_.lock();
  cmd_vel_ = *msg;
  cmd_vel_mtx_.unlock();
}

void rc::Twist::CurStateCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
  current_state_mtx_.lock();
  current_state_ = *msg;
  current_state_mtx_.unlock();
}

void rc::Twist::CtrlUpdate() {
  ros::Rate r(pub_rate_);
  //std_msgs::Float32 left_ctrl_msg;
  //std_msgs::Float32 right_ctrl_msg;
  
  while (ros::ok()) {
    ros::spinOnce();

    // get current state
    double current_vel = 0;
    double current_ang_vel = 0;
    current_state_mtx_.lock();
    current_vel = current_state_.vector.x;
    current_ang_vel = current_state_.vector.z;
    // current_vel = sqrt(pow(current_state_.twist.twist.linear.x, 2) +
    //                    pow(current_state_.twist.twist.linear.y, 2));
    // current_ang_vel= current_state_.twist.twist.angular.z;
    current_state_mtx_.unlock();

    // get objective velocity
    double obj_vel = 0;
    double obj_ang_vel = 0;
    cmd_vel_mtx_.lock();
    obj_vel = cmd_vel_.linear.x;
    obj_ang_vel = cmd_vel_.angular.z;
    cmd_vel_mtx_.unlock();

    ROS_DEBUG_STREAM_NAMED("statement", "objective velocity : " << obj_vel);
    ROS_DEBUG_STREAM_NAMED("statement", "current velocity : " << current_vel);

    double err_vel = 0;
    double err_int_vel = 0;
    double ctrl_vel = 0;

    double err_ang_vel = 0;
    double err_int_ang_vel = 0;
    double ctrl_ang_vel = 0;

    err_vel = obj_vel - current_vel;
    err_int_vel += err_vel;

    //ROS_INFO("err_vel : %lf", err_vel);

    err_ang_vel = obj_ang_vel - current_ang_vel;
    err_int_ang_vel += err_ang_vel;

    // PI Controller
    ctrl_vel = kp_vel_ * err_vel + ki_vel_ * err_int_vel;
    ctrl_ang_vel = kp_ang_vel_ * err_ang_vel + ki_ang_vel_ * err_int_ang_vel;

    // Normalize
    double ctrl_left = ctrl_vel - (tread_/2.0)*ctrl_ang_vel;
    double ctrl_right = ctrl_vel + (tread_/2.0)*ctrl_ang_vel;
    ctrl_left /= max_left_vel_;
    ctrl_right /= max_right_vel_;

    std_msgs::Float64MultiArray cmd_msg;
    cmd_msg.data.push_back(ctrl_left);
    cmd_msg.data.push_back(0);
    cmd_msg.data.push_back(ctrl_right);
    cmd_msg.data.push_back(0);
    motor_command_pub_.publish(cmd_msg);
    /*
    left_ctrl_msg.data = ctrl_left;
    right_ctrl_msg.data = ctrl_right;

    ROS_INFO("left_ctrl : %lf", left_ctrl_msg.data);
    ROS_INFO("right_ctrl : %lf", right_ctrl_msg.data);
    */
    
    // publish the ctrl msg
    //left_control_msg_pub_.publish(left_ctrl_msg);
    //right_control_msg_pub_.publish(right_ctrl_msg);
    
    r.sleep();
  }
}

  
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robotx_twist_controller_node");
  rc::Twist rc_twist;
  rc_twist.CtrlUpdate();

  return 0;
}
