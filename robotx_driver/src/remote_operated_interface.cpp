// headers in this package
#include <remote_operated_interface.h>
#include <robotx_hardware_interface.h>

remote_operated_interface::remote_operated_interface(
    std::function<void(std_msgs::Float64MultiArray motor_command)> send_motor_command)
    : params_() {
  send_motor_command_ = send_motor_command;
  send_motor_command_signal_.connect(send_motor_command_);
  trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>("/robotx_state_machine_node/control_state_machine/trigger_event", 1);
  current_state_sub_ = nh_.subscribe("/robotx_state_machine_node/control_state_machine/current_state", 1, &remote_operated_interface::current_state_callback_, this);
  joy_sub_ = nh_.subscribe("/joy", 1, &remote_operated_interface::joy_callback_, this);
}

remote_operated_interface::~remote_operated_interface() {}

void remote_operated_interface::current_state_callback_(robotx_msgs::State msg){
  std::lock_guard<std::mutex> lock(mtx_);
  current_state_ = msg;
  return;
}

void remote_operated_interface::joy_callback_(sensor_msgs::Joy msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  last_joy_cmd_ = msg;
  std_msgs::Float64MultiArray motor_command_msg;
  if (params_.controller_type == params_.DUALSHOCK4_SIMPLE) {
    if (last_joy_cmd_.buttons[12] == 1) {
      if(current_state_.current_state == "autonomous"){
        robotx_msgs::Event event;
        event.trigger_event_name = "manual_override";
        trigger_event_pub_.publish(event);
      }
      if(current_state_.current_state == "remote_operated"){
        robotx_msgs::Event event;
        event.trigger_event_name = "system_bringup";
        trigger_event_pub_.publish(event);
      }
    }
    motor_command_msg.data = std::vector<double>(4);
    motor_command_msg.data[0] = last_joy_cmd_.axes[1];
    motor_command_msg.data[1] = 0;
    motor_command_msg.data[2] = last_joy_cmd_.axes[5];
    motor_command_msg.data[3] = 0;
    send_motor_command_signal_(motor_command_msg);
  }
  if (params_.controller_type == params_.DUALSHOCK4) {
    if (last_joy_cmd_.buttons[12] == 1) {
      if(current_state_.current_state == "autonomous"){
        robotx_msgs::Event event;
        event.trigger_event_name = "manual_override";
        trigger_event_pub_.publish(event);
      }
      if(current_state_.current_state == "remote_operated"){
        robotx_msgs::Event event;
        event.trigger_event_name = "system_bringup";
        trigger_event_pub_.publish(event);
      }
    }
    motor_command_msg.data = std::vector<double>(4);
    motor_command_msg.data[0] = last_joy_cmd_.axes[3];
    motor_command_msg.data[1] = -last_joy_cmd_.axes[0];
    motor_command_msg.data[2] = last_joy_cmd_.axes[4];
    motor_command_msg.data[3] = -last_joy_cmd_.axes[2];
    send_motor_command_signal_(motor_command_msg);
  }
  if (params_.controller_type == params_.LOGICOOL) {
    if (last_joy_cmd_.buttons[8] == 1) {
      if(current_state_.current_state == "autonomous"){
        robotx_msgs::Event event;
        event.trigger_event_name = "manual_override";
        trigger_event_pub_.publish(event);
      }
      if(current_state_.current_state == "remote_operated"){
        robotx_msgs::Event event;
        event.trigger_event_name = "system_bringup";
        trigger_event_pub_.publish(event);
      }
    }
    motor_command_msg.data = std::vector<double>(4);
    motor_command_msg.data[0] = last_joy_cmd_.axes[1];
    motor_command_msg.data[1] = last_joy_cmd_.axes[0];
    motor_command_msg.data[2] = last_joy_cmd_.axes[3];
    motor_command_msg.data[3] = last_joy_cmd_.axes[2];
    send_motor_command_signal_(motor_command_msg);
  }
  return;
}
