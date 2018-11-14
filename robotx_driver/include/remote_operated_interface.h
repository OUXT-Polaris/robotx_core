#ifndef REMOTE_OPERATED_INTERFACE_H_INCLUDED
#define REMOTE_OPERATED_INTERFACE_H_INCLUDED

// headers in this package
#include <robotx_msgs/State.h>
#include <robotx_msgs/Event.h>
#include <joy_analyzer.h>

// headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

// headers in STL
#include <functional>
#include <mutex>

// headers in boost
#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <boost/system/error_code.hpp>

/**
 * @brief interface for remote operation
 *
 */
class remote_operated_interface {
 public:
  /**
   * @brief remote_operated_interface class parameters
   *
   */
  struct parameters {
    enum controllers { DUALSHOCK4_SIMPLE = 0, DUALSHOCK4 = 1, LOGICOOL = 2 };
    int controller_type;
    /**
     * @brief Construct a new parameters objects
     *
     */
    parameters() {
      ros::param::param<int>(ros::this_node::getName() + "/controller_type", controller_type,
                             DUALSHOCK4_SIMPLE);
    }
  };

  /**
   * @brief Construct a new remote operated interface object
   *
   * @param set_action_mode_function callback function for set_action_mode
   */
  remote_operated_interface(std::function<void(std_msgs::Float64MultiArray)> send_motor_command);
  /**
   * @brief Destroy the remote operated interface object
   *
   */
  ~remote_operated_interface();

 private:
  /**
   * @brief analyzer for joystick
   */
  joy_analyzer analyzer_;
  /**
   * @brief ROS nodehandle
   *
   */
  ros::NodeHandle nh_;
  /**
   * @brief ROS Subscriber for /joy(message type: sensor_msgs/Joy)
   *
   */
  ros::Subscriber joy_sub_;
  /**
   * @brief ROS Subscriber for /robotx_state_machine_node/control_state_machine/current_state
   * (message type: robotx_msgs/State)
   */
  ros::Subscriber current_state_sub_;
  /**
   * @brief parameters for remote_operated_interface class
   *
   */
  const parameters params_;
  /**
   * @brief callback function for set_action_mode
   *
   */
  std::function<void(robotx_msgs::Event)> set_action_mode_function_;
  /**
   * @brief singal for set_action_mode function
   * @sa robotx_hardware_interface::set_action_mode
   */
  boost::signals2::signal<void(robotx_msgs::Event)> action_mode_signal_;
  /**
   * @brief callback function for sending motor command
   * @sa robotx_hardware_interface::recieve_remote_oprated_motor_command
   */
  std::function<void(std_msgs::Float64MultiArray motor_command)> send_motor_command_;
  /**
   * @brief signal for send_motor_command function
   * @sa robotx_hardware_interface::send_motor_command_
   */
  boost::signals2::signal<void(std_msgs::Float64MultiArray motor_command)> send_motor_command_signal_;
  /**
   * @brief ROS callback function for joystick
   *
   * @param msg joystick command.
   */
  void joy_callback_(sensor_msgs::Joy msg);
  /**
   * @brief parameters for last joysick command.
   *
   */
  sensor_msgs::Joy last_joy_cmd_;
  void current_state_callback_(robotx_msgs::State msg);
  ros::Publisher trigger_event_pub_;
  robotx_msgs::State current_state_;
  std::mutex mtx_;
};

#endif  // REMOTE_OPERATED_INTERFACE_H_INCLUDED
