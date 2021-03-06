#include <serial_sender.h>

serial_sender::serial_sender()
{
    pnh_ = ros::NodeHandle("~");
    pnh_.param<std::string>("port", port_, "");
    pnh_.param<int>("baud_rate", baud_rate_, 9600);
    pnh_.param<int>("baud_rate", baud_rate_, 9600);
    pnh_.param<double>("send_rate", send_rate_, 10.0);
    message_sub_ = pnh_.subscribe("msg",1,&serial_sender::message_callback_,this);
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
	port.set_option(serial_port_base::baud_rate(baud_rate_));
	port.set_option(serial_port_base::character_size(8));
	port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
	port.set_option(serial_port_base::parity(serial_port_base::parity::none));
	port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    char rbuf[32];
	std::size_t length;
    while(ros::ok())
    {
        ROS_INFO_STREAM("send serial to " << port_ << ":" << write_buf_);
        try
        {
            port.write_some(buffer(write_buf_));
        }
        catch(...)
        {
            ROS_ERROR_STREAM("failed to write");
        }
        //length = port.read_some(buffer(rbuf));
        //std::cout.write(rbuf, length);
        rate.sleep();
    }
    return;
}