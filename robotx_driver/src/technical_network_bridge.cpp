#include <technical_network_bridge.h>

// hearers in stl
#include <stdio.h>
#include <string.h>

technical_network_bridge::technical_network_bridge() {
  message_recieved_ = false;
  nh_.param<std::string>(ros::this_node::getName() + "/team_id", team_id_, "OUXT Polaris");
  nh_.param<std::string>(ros::this_node::getName() + "/ip_address", ip_address_, "127.0.0.1");
  nh_.param<int>(ros::this_node::getName() + "/port", port_, 31400);
  nh_.param<double>(ros::this_node::getName() + "/heartbeat_publish_rate", heartbeat_publish_rate_, 1);
  client_ = new tcp_client(io_service_, ip_address_, port_);
  io_service_.run();
  connection_status_pub_ = nh_.advertise<robotx_msgs::TechnicalDirectorNetworkStatus>(
      ros::this_node::getName() + "/connection_status", 1);
  heartbeat_sub_ = nh_.subscribe("/heartbeat", 1, &technical_network_bridge::heartbeat_callback, this);
  entrance_and_exit_gates_report_sub_ = 
    nh_.subscribe("/entrance_and_exit_gates_report",1,&technical_network_bridge::entrance_and_exit_gates_report_callback_,this);
  identify_symbols_and_dock_report_sub_ = 
    nh_.subscribe("/identify_symbols_and_dock_report",1,&technical_network_bridge::identify_symbols_and_dock_report_callback_,this);
}

technical_network_bridge::~technical_network_bridge() {}

void technical_network_bridge::run(){
  tcp_thread = boost::thread(&technical_network_bridge::publish_heartbeat_message, this);
}

void technical_network_bridge::entrance_and_exit_gates_report_callback_(const robotx_msgs::EntranceAndExitGatesReport::ConstPtr &msg)
{
  std::string tcp_send_msg_;
  tcp_send_msg_ = "$RXGAT";
  std::string hh,mm,ss;
  get_local_time_(hh,mm,ss);
  tcp_send_msg_ = tcp_send_msg_ + hh + mm + ss;
  return;
}

void technical_network_bridge::identify_symbols_and_dock_report_callback_(const robotx_msgs::IdentifySymbolsAndDockReport::ConstPtr &msg)
{
  return;
}

void technical_network_bridge::publish_heartbeat_message() {
  ros::Rate loop_rate(heartbeat_publish_rate_);
  while (ros::ok()) {
    publish_connection_status_message();
    mtx_.lock();
    if (message_recieved_ == true) {
      client_->send(heartbeat_tcp_send_msg_);
    }
    mtx_.unlock();
    loop_rate.sleep();
  }
}

void technical_network_bridge::heartbeat_callback(const robotx_msgs::Heartbeat::ConstPtr &msg) {
  message_recieved_ = true;
  mtx_.lock();
  heartbeat_msg_ = *msg;
  update_heartbeat_message();
  mtx_.unlock();
}

std::string technical_network_bridge::generate_checksum(const char *data) {
  int crc = 0;
  int i;
  // the first $ sign and the last two bytes of original CRC + the * sign
  for (i = 1; i < strlen(data) - 3; i++) {
    crc ^= data[i];
  }
  std::string checksum = "";
  std::stringstream ss;
  ss << std::hex << crc;
  std::dec;
  return checksum;
}

void technical_network_bridge::update_heartbeat_message() {
  std::string heartbeat_tcp_send_msg_;
  heartbeat_tcp_send_msg_ = "$RXHRT,";
  std::string hh,mm,ss;
  get_local_time_(hh,mm,ss);
  heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + hh + mm + ss;
  heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + std::to_string(heartbeat_msg_.latitude) + ",";
  if (heartbeat_msg_.north_or_south == heartbeat_msg_.NORTH) {
    heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + "N,";
  } else {
    heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + "S,";
  }
  heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + std::to_string(heartbeat_msg_.longitude) + ",";
  if (heartbeat_msg_.east_or_west == heartbeat_msg_.EAST) {
    heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + "E,";
  } else {
    heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + "W,";
  }
  heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + team_id_;
  heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + std::to_string(heartbeat_msg_.vehicle_mode) + ",";
  heartbeat_tcp_send_msg_ = heartbeat_tcp_send_msg_ + std::to_string(heartbeat_msg_.current_task_number);
  heartbeat_tcp_send_msg_ = "$" + heartbeat_tcp_send_msg_ + "*" + generate_checksum(heartbeat_tcp_send_msg_.c_str());
}

void technical_network_bridge::publish_connection_status_message() {
  bool connection_status = client_->get_connection_status();
  robotx_msgs::TechnicalDirectorNetworkStatus connection_status_msg;
  if (connection_status == true) {
    connection_status_msg.status = connection_status_msg.CONNECTED;
  } else {
    connection_status_msg.status = connection_status_msg.CONNECTION_LOST;
  }
  connection_status_msg.port = port_;
  connection_status_msg.ip_address = ip_address_;
  connection_status_pub_.publish(connection_status_msg);
}

void technical_network_bridge::get_local_time_(std::string& hst_hh, std::string& hst_mm, std::string& hst_ss)
{
  time_t t;
  struct tm *tm;
  tm = localtime(&t);
  if (tm->tm_hour < 9)
    hst_hh = "0" + std::to_string(tm->tm_hour);
  else
    hst_hh = std::to_string(tm->tm_hour);
  if (tm->tm_min < 9)
    hst_mm = "0" + std::to_string(tm->tm_min);
  else
    hst_mm = std::to_string(tm->tm_min);
  if (tm->tm_sec < 9)
    hst_ss = "0" + std::to_string(tm->tm_sec);
  else
    hst_ss = std::to_string(tm->tm_sec);
}

