#include <battery_monitor_driver.h>

battery_monitor_driver::battery_monitor_driver(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("ip_address", ip_address_, "127.0.0.1");
    pnh_.param<int>("port", port_, 5000);
    std::function<void(std::string)> callback_func = std::bind(&battery_monitor_driver::publish_battery_state_,this);
    tcp_server_ptr_ =  boost::make_shared<tcp_server>(port_, io_service_, callback_func);
}

battery_monitor_driver::~battery_monitor_driver()
{

}

void battery_monitor_driver::publish_battery_state_()
{
    return;
}