#include <dynamixel_driver.h>

dynamixel_driver::dynamixel_driver()
{
    ros::param::param<std::string>(ros::this_node::getName() + "/input_topic", input_topic_, ros::this_node::getName()+"/target_angle");
    ros::param::param<int>(ros::this_node::getName() + "/id", motor_id_, 0);
    joint_command_client_ = nh_.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");
    command_sub_ = nh_.subscribe(input_topic_,10,&dynamixel_driver::command_callback_,this);
}

dynamixel_driver::~dynamixel_driver()
{

}

void dynamixel_driver::command_callback_(std_msgs::Float64 msg)
{
    dynamixel_workbench_msgs::JointCommand joint_command;
    joint_command.request.unit = "raw";
    joint_command.request.id = (uint8_t)motor_id_;
    joint_command.request.goal_position = msg.data;
    if(!joint_command_client_.call(joint_command))
    {
        ROS_ERROR_STREAM("failed to write target angle to " << motor_id_  << " dynamixel.");
    }
    return;
}