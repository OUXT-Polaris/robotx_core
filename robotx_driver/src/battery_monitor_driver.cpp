#include <battery_monitor_driver.h>

battery_monitor_driver::battery_monitor_driver(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("ip_address", ip_address_, "127.0.0.1");
    pnh_.param<int>("port", port_, 5000);
    control_battery_pub_ = pnh_.advertise<sensor_msgs::BatteryState>("control_battery/state",1);
    motor_battery_pub_ = pnh_.advertise<sensor_msgs::BatteryState>("motor_battery/state",1);
    power_state_pub_ = pnh_.advertise<robotx_msgs::PowerStatus>("power/status",1);
    std::function<void(std::string)> callback_func = std::bind(&battery_monitor_driver::publish_battery_state_,this, std::placeholders::_1);
    tcp_server_ptr_ =  boost::make_shared<tcp_server>(port_, io_service_, callback_func);
}

battery_monitor_driver::~battery_monitor_driver()
{

}

void battery_monitor_driver::publish_battery_state_(std::string data)
{
    robotx_msgs::PowerStatus power_status_msg;
    ros::Time now = ros::Time::now();
    using namespace boost::property_tree;
    ptree pt;
    read_json(data, pt);
    if(boost::optional<float> voltage = pt.get_optional<float>("volt.ctrlbat"))
    {
        if(boost::optional<float> amp = pt.get_optional<float>("amp.ctrl"))
        {
            sensor_msgs::BatteryState battery_msg;
            battery_msg.header.stamp = now;
            battery_msg.voltage = *voltage;
            battery_msg.current = *amp * -1;
            battery_msg.location = "control main battery";
            control_battery_pub_.publish(battery_msg);
        }
    }
    if(boost::optional<float> voltage = pt.get_optional<float>("volt.motorbat"))
    {
        if(boost::optional<float> amp = pt.get_optional<float>("amp.motor"))
        {
            sensor_msgs::BatteryState battery_msg;
            battery_msg.header.stamp = now;
            battery_msg.voltage = *voltage;
            battery_msg.current = *amp * -1;
            battery_msg.location = "motor main battery";
            motor_battery_pub_.publish(battery_msg);
        }
    }
    if(boost::optional<std::string> value = pt.get_optional<std::string>("volt.ctrl"))
    {
        if(*value == "ON")
        {
            power_status_msg.control_power_supply_status = power_status_msg.ACTIVE;
        }
        else if(*value == "OFF")
        {
            power_status_msg.control_power_supply_status = power_status_msg.INACTIVE;
        }
        else
        {
            power_status_msg.control_power_supply_status = power_status_msg.UNKOWN;
        }
    }
    if(boost::optional<std::string> value = pt.get_optional<std::string>("volt.motor"))
    {
        if(*value == "ON")
        {
            power_status_msg.motor_power_supply_status = power_status_msg.ACTIVE;
        }
        else if(*value == "OFF")
        {
            power_status_msg.motor_power_supply_status = power_status_msg.INACTIVE;
        }
        else
        {
            power_status_msg.motor_power_supply_status = power_status_msg.UNKOWN;
        }
    }
    if(boost::optional<float> value = pt.get_optional<float>("volt.ctrl24v"))
    {
        power_status_msg.control_24v_voltage = *value;
    }
    if(boost::optional<float> value = pt.get_optional<float>("volt.ctrl12v"))
    {
        power_status_msg.control_12v_voltage = *value;
    }
    if(boost::optional<float> value = pt.get_optional<float>("volt.ctrl5v"))
    {
        power_status_msg.control_5v_voltage = *value;
    }
    if(boost::optional<float> value = pt.get_optional<float>("volt.motor12v"))
    {
        power_status_msg.motor_12v_voltage = *value;
    }
    power_state_pub_.publish(power_status_msg);
    return;
}