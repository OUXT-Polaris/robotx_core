#include <serial_sender.h>

serial_sender::serial_sender()
{
    pnh_ = ros::NodeHandle("~");
    pnh_.param<std::string>("port", port_, "");
    pnh_.param<int>("baud_rate", baud_rate_, 9600);
    pnh_.param<double>("send_rate", send_rate_, 10.0);
    message_sub_ = nh_.subscribe("msg",1,&serial_sender::message_callback_,this);
}

serial_sender::~serial_sender()
{

}

void serial_sender::run()
{
    boost::thread send_thread_ = boost::thread(&serial_sender::send_,this);
    return;
}

void serial_sender::message_callback_(const std_msgs::StringConstPtr msg)
{
    write_buf_ = msg->data;
    return;
}

void serial_sender::send_()
{
    ros::Rate rate = ros::Rate(send_rate_);
    io_service io;
	serial_port port(io, port_.c_str());
	port.set_option(serial_port_base::baud_rate(19200));
	port.set_option(serial_port_base::character_size(8));
	port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
	port.set_option(serial_port_base::parity(serial_port_base::parity::none));
	port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    while(ros::ok())
    {
        port.write_some(buffer(write_buf_));
        rate.sleep();
    }
    return;
}