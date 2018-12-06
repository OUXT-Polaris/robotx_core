#include <signal_lamp_driver.h>

singal_lamp_driver::singal_lamp_driver(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("cmd_topic", cmd_topic_, ros::this_node::getName()+"/cmd");
    pnh_.param<std::string>("ip_address", ip_address_, "127.0.0.1");
    pnh_.param<int>("port", port_, 5000);
    pnh_.param<double>("timeout", timeout_, 10.0);
    pnh_.param<double>("publish_rate", publish_rate_, 10.0);
    tcp_client_ptr_ = boost::make_shared<tcp_client>(io_service_, ip_address_, port_, timeout_);
    io_service_.run();
    command_sub_ = nh_.subscribe(cmd_topic_,1,&singal_lamp_driver::command_callback_,this);
}

singal_lamp_driver::~singal_lamp_driver()
{

}

void singal_lamp_driver::command_callback_(const robotx_msgs::SignalLamp::ConstPtr msg)
{
    mtx_.lock();
    command_ = *msg;
    mtx_.unlock();
    return;
}

void singal_lamp_driver::run()
{
    boost::thread cmd_thread = boost::thread(&singal_lamp_driver::publish_cmd_,this);
    return;
}

void singal_lamp_driver::publish_cmd_()
{
    using namespace boost::property_tree;
    std::stringstream ss;
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        if(command_)
        {
            mtx_.unlock();
            rate.sleep();
            continue;
        }
        ptree pt;
        if(command_->mode == robotx_msgs::SignalLamp::MODE_AUTO)
        {
            pt.put("mode", "auto");
        }
        else if(command_->mode == robotx_msgs::SignalLamp::MODE_MANUAL)
        {
            pt.put("mode", "manual");
        }
        else if(command_->mode == robotx_msgs::SignalLamp::MODE_EMERGENCY)
        {
            pt.put("mode", "emergency");
        }
        else
        {
            mtx_.unlock();
            rate.sleep();
            continue;
        }
        if(command_->value >= 0 && 11>=command_->value)
        {
            pt.put("value", command_->value);
        }
        else
        {
            mtx_.unlock();
            rate.sleep();
            continue;
        }
        write_json(ss,pt);
        tcp_client_ptr_->send(ss.str());
        mtx_.unlock();
        rate.sleep();
    }
    return;
}