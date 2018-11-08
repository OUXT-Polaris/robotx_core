#include <robotx_navigation_planner.h>

robotx_navigation_planner::robotx_navigation_planner()
{
    obstacle_avoid_cmd_ = boost::none;
    waypoint_planner_cmd_ = boost::none;
    ros::param::param<std::string>(ros::this_node::getName() + "/current_state_topic", 
        current_state_topic_, "/robotx_state_machine_node/navigation_state_machine/current_state");
    ros::param::param<std::string>(ros::this_node::getName() + "/trigger_event_topic", 
        trigger_event_topic_, "/robotx_state_machine_node/navigation_state_machine/trigger_event");
    ros::param::param<std::string>(ros::this_node::getName() + "/waypoint_planner_cmd_topic", 
        waypoint_planner_cmd_topic_, "/carrot_planner_node/cmd_vel");
    ros::param::param<std::string>(ros::this_node::getName() + "/obstacle_avoid_cmd_topic", 
        obstacle_avoid_cmd_topic_, "/obstacle_avoid_node/cmd_vel");
    ros::param::param<double>(ros::this_node::getName() + "/publish_rate", publish_rate_, 30.0);
    trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>(trigger_event_topic_, 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    current_state_sub_ = nh_.subscribe(current_state_topic_, 10, &robotx_navigation_planner::current_state_callback_, this);
    waypoint_planner_cmd_sub_ = nh_.subscribe(waypoint_planner_cmd_topic_, 10, &robotx_navigation_planner::waypoint_planner_cmd_callback_, this);
    obstacle_avoid_cmd_sub_ =  nh_.subscribe(obstacle_avoid_cmd_topic_, 10, &robotx_navigation_planner::obstacle_avoid_cmd_callback_, this);
}

robotx_navigation_planner::~robotx_navigation_planner()
{

}

void robotx_navigation_planner::publish_cmd_vel_()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return;
}

void robotx_navigation_planner::run()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        publish_cmd_vel_();
        rate.sleep();
    }
    return;
}

void robotx_navigation_planner::current_state_callback_(const robotx_msgs::StateConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    return;
}

void robotx_navigation_planner::waypoint_planner_cmd_callback_(const geometry_msgs::Twist msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    waypoint_planner_cmd_ = msg;
    return;
}

void robotx_navigation_planner::obstacle_avoid_cmd_callback_(const geometry_msgs::Twist msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    obstacle_avoid_cmd_ = msg;
    return;
}