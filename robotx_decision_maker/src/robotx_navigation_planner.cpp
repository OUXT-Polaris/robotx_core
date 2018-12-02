#include <robotx_navigation_planner.h>

robotx_navigation_planner::robotx_navigation_planner()
{
    obstacle_avoid_cmd_ = boost::none;
    waypoint_planner_cmd_ = boost::none;
    current_state_ = boost::none;
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
    state_changed_sub_ = nh_.subscribe("/robotx_state_machine_node/navigation_state_machine/state_changed", 10, &robotx_navigation_planner::state_changed_callback_, this);
}

robotx_navigation_planner::~robotx_navigation_planner()
{

}

void robotx_navigation_planner::publish_cmd_vel_()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        if(current_state_)
        {
            if(current_state_->current_state == "navigation_start"
                || current_state_->current_state == "navigation_finished")
            {
                mtx_.unlock();
                rate.sleep();
                continue;
            }
            if(current_state_->current_state == "go_straight")
            {
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.5;
                cmd_vel_pub_.publish(cmd);
                mtx_.unlock();
                rate.sleep();
                continue;
            }
            if(current_state_->current_state == "obstacle_avoid")
            {
                if(obstacle_avoid_cmd_)
                {
                    cmd_vel_pub_.publish(*obstacle_avoid_cmd_);
                    mtx_.unlock();
                    rate.sleep();
                    continue;
                }
                else
                {
                    geometry_msgs::Twist empty_cmd;
                    cmd_vel_pub_.publish(empty_cmd);
                    mtx_.unlock();
                    rate.sleep();
                    continue;
                }
            }
            if(current_state_->current_state == "heading_to_next_waypoint"
                || current_state_->current_state == "moving_to_next_waypoint"
                || current_state_->current_state == "align_to_next_waypoint")
            {
                if(waypoint_planner_cmd_)
                {
                    cmd_vel_pub_.publish(*waypoint_planner_cmd_);
                    mtx_.unlock();
                    rate.sleep();
                    continue;
                }
            }
            geometry_msgs::Twist empty_cmd;
            cmd_vel_pub_.publish(empty_cmd);
        }
        mtx_.unlock();
        rate.sleep();
    }
    return;
}

void robotx_navigation_planner::state_changed_callback_(robotx_msgs::StateChanged msg)
{
    if(msg.current_state == "go_straight")
    {
        boost::thread timer_thread(&robotx_navigation_planner::publish_timer_event_,this);
    }
    return;
}

void robotx_navigation_planner::publish_timer_event_()
{
    ros::Duration sleep_time(5);
    sleep_time.sleep();
    robotx_msgs::Event event;
    event.header.stamp = ros::Time::now();
    if(current_state_ && current_state_->current_state == "go_straight")
    {
        event.trigger_event_name = "go_straight_finished";
        trigger_event_pub_.publish(event);
    }
    return;
}

void robotx_navigation_planner::run()
{
    boost::thread publish_cmd_vel_th = boost::thread(&robotx_navigation_planner::publish_cmd_vel_, this);
    return;
}

void robotx_navigation_planner::current_state_callback_(robotx_msgs::State msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_state_ = msg;
    return;
}

void robotx_navigation_planner::waypoint_planner_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    waypoint_planner_cmd_ = *msg;
    return;
}

void robotx_navigation_planner::obstacle_avoid_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    obstacle_avoid_cmd_ = *msg;
    return;
}