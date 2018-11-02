#include <rostate_machine.h>

rostate_machine::rostate_machine(std::string xml_filepath, std::string dot_filepath, std::string state_machine_name)
{
    state_machine_ptr_ = std::make_shared<state_machine>(xml_filepath);
    state_machine_ptr_->draw_state_machine(dot_filepath);
    state_machine_name_ = state_machine_name;
    nh_.param<double>(ros::this_node::getName()+"/publish_rate", publish_rate_, 1);
    current_state_pub_ = nh_.advertise<robotx_msgs::State>(ros::this_node::getName()+"/"+state_machine_name+"/current_state",1);
}

rostate_machine::~rostate_machine()
{

}

void rostate_machine::publish_current_state_()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        robotx_msgs::State state_msg;
        state_msg.current_state = state_machine_ptr_->get_current_state();
        rate.sleep();
    }
    return;
}