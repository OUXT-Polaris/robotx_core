#include <rostate_machine.h>

rostate_machine::rostate_machine(std::string xml_filepath, std::string dot_filepath, std::string state_machine_name)
{
    state_machine_ptr_ = std::make_shared<state_machine>(xml_filepath);
    state_machine_ptr_->draw_state_machine(dot_filepath);
    state_machine_name_ = state_machine_name;
    nh_.param<double>(ros::this_node::getName()+"/publish_rate", publish_rate_, 10);
    current_state_pub_ = nh_.advertise<robotx_msgs::State>(ros::this_node::getName()+"/"+state_machine_name+"/current_state",1);
}

rostate_machine::~rostate_machine()
{

}

void rostate_machine::event_callback_(robotx_msgs::Event msg)
{
    bool result = state_machine_ptr_->try_transition(msg.trigger_event_name);
    if(!result)
    {
        state_info_t info = state_machine_ptr_->get_state_info();
        ROS_ERROR_STREAM("failed to transition, current state : "<< info.current_state << ",event_name : " << msg.trigger_event_name);
    }
    return;
}

void rostate_machine::run()
{
    boost::thread publish_thread(boost::bind(&rostate_machine::publish_current_state_, this));
    trigger_event_sub_ = nh_.subscribe<robotx_msgs::Event>(ros::this_node::getName()+"/"+state_machine_name_+"/trigger_event", 10, &rostate_machine::event_callback_,this);
    return;
}

void rostate_machine::publish_current_state_()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        robotx_msgs::State state_msg;
        state_info_t info = state_machine_ptr_->get_state_info();
        state_msg.current_state = info.current_state;
        state_msg.possible_transitions = info.possibe_transitions;
        state_msg.possible_transition_states = info.possibe_transition_states;
        state_msg.header.stamp = ros::Time::now();
        current_state_pub_.publish(state_msg);
        rate.sleep();
    }
    return;
}