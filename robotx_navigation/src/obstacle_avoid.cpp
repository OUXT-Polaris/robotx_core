#include <obstacle_avoid.h>

obstacle_avoid::obstacle_avoid() : tf_listener_(tf_buffer_)
{
    current_state_ = boost::none;
    map_recieved_ = false;
    odom_recieved_ = false;
    twist_cmd_recieved_ = false;
    nh_.param<std::string>(ros::this_node::getName()+"/map_topic", map_topic_, "/obstacle_map");
    nh_.param<std::string>(ros::this_node::getName()+"/cmd_vel_topic", cmd_vel_topic_, ros::this_node::getName()+"/cmd_vel");
    nh_.param<std::string>(ros::this_node::getName()+"/raw_cmd_vel_topic", raw_cmd_vel_topic_, ros::this_node::getName()+"/input_cmd_vel");
    nh_.param<std::string>(ros::this_node::getName()+"/odom_topic", odom_topic_, "/odom");
    nh_.param<std::string>(ros::this_node::getName()+"/robot_frame", robot_frame_, "base_link");
    nh_.param<std::string>(ros::this_node::getName()+"/target_pose_topic", target_pose_topic_, ros::this_node::getName()+"/target_pose");
    nh_.param<std::string>(ros::this_node::getName()+"/current_state_topic", current_state_topic_, ros::this_node::getName()+"current_state");
    nh_.param<std::string>(ros::this_node::getName()+"/trigger_event_topic", trigger_event_topic_, ros::this_node::getName()+"/trigger_event");
    nh_.param<double>(ros::this_node::getName()+"/search_angle", search_angle_, 0.3);
    nh_.param<double>(ros::this_node::getName()+"/search_radius", search_radius_, 8.0);
    nh_.param<double>(ros::this_node::getName()+"/search_radius_side", search_radius_side_, 5.0);
    nh_.param<double>(ros::this_node::getName()+"/search_radius_behind", search_radius_behind_, 3.0);
    nh_.param<double>(ros::this_node::getName()+"/search_angle_behind", search_angle_, 0.3);
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    nearest_obstacle_range_pub_ = nh_.advertise<std_msgs::Float32>(ros::this_node::getName()+"/nearest_obstacle_range", 10);
    trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>(trigger_event_topic_, 1);
    map_sub_ = nh_.subscribe(map_topic_, 3, &obstacle_avoid::obstacle_map_callback_, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &obstacle_avoid::odom_callback_, this);
    target_pose_sub_ = nh_.subscribe(target_pose_topic_, 10, &obstacle_avoid::target_pose_callback_, this);
    current_state_sub_ = nh_.subscribe(current_state_topic_, 10, &obstacle_avoid::current_state_callback_, this);
}

obstacle_avoid::~obstacle_avoid()
{

}

void obstacle_avoid::current_state_callback_(robotx_msgs::State msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_state_ = msg;
    if(!obstacle_found_() && current_state_->current_state == "obstacle_avoid")
    {
        robotx_msgs::Event event_msg;
        event_msg.trigger_event_name = "obstacle_not_found";
        event_msg.header.stamp = ros::Time::now();
        trigger_event_pub_.publish(event_msg);
    }
    if(!obstacle_found_() && current_state_->current_state == "turn_right")
    {
        robotx_msgs::Event event_msg;
        event_msg.trigger_event_name = "obstacle_not_found";
        event_msg.header.stamp = ros::Time::now();
        trigger_event_pub_.publish(event_msg);
    }
    if(!obstacle_found_() && current_state_->current_state == "turn_left")
    {
        robotx_msgs::Event event_msg;
        event_msg.trigger_event_name = "obstacle_not_found";
        event_msg.header.stamp = ros::Time::now();
        trigger_event_pub_.publish(event_msg);
    }
    return;
}

void obstacle_avoid::target_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_pose_ = *msg;
    return;
}

bool obstacle_avoid::obstacle_found_()
{
    for(int i=0; i<map_.points.size(); i++)
    {
        double yaw = std::atan2(map_.points[i].y,map_.points[i].x);
        double dist = std::sqrt(map_.points[i].x*map_.points[i].x + map_.points[i].y*map_.points[i].y);
        if(std::fabs(search_angle_) > std::fabs(yaw))
        {
            if(search_radius_ > dist)
            {
                return true;
            }
        }
        else if(yaw > (M_PI-search_angle_behind_) || yaw > (-M_PI+search_angle_behind_) )
        {
            if(search_radius_behind_ > dist)
            {
                return true;
            }
        }
        else
        {
            if(search_radius_side_ > dist)
            {
                return true;
            }
        }
    }
    return false;
}

void obstacle_avoid::publish_nearest_obstacle_range_()
{
    std_msgs::Float32 nearest_obstacle_range;
    nearest_obstacle_range.data = -1;
    for(auto obstacle_itr = map_.points.begin(); obstacle_itr != map_.points.end(); obstacle_itr++)
    {
        double dist = std::sqrt(obstacle_itr->x*obstacle_itr->x + obstacle_itr->y*obstacle_itr->y);
        if(nearest_obstacle_range.data == -1)
        {
            nearest_obstacle_range.data = dist;
        }
        else if(nearest_obstacle_range.data < dist)
        {
            nearest_obstacle_range.data = dist;
        }
    }
    nearest_obstacle_range_pub_.publish(nearest_obstacle_range);
    return;
}

void obstacle_avoid::odom_callback_(const nav_msgs::Odometry::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_recieved_ = true;
    odom_ = *msg;
    if(!current_state_)
    {
        return;
    }
    if(current_state_->current_state == "stacked")
    {
        geometry_msgs::Twist stop_cmd;
        twist_cmd_pub_.publish(stop_cmd);
        int num_left_obstacles = 0;
        int num_right_obstacles = 0;
        for(auto obstacle_itr = map_.points.begin(); obstacle_itr != map_.points.end(); obstacle_itr++)
        {
            if(obstacle_itr->y > 0)
            {
                num_left_obstacles++;
            }
            else
            {
                num_right_obstacles++;
            }
        }
        if(num_left_obstacles > num_right_obstacles)
        {
            robotx_msgs::Event event_msg;
            event_msg.header.stamp = ros::Time::now();
            event_msg.trigger_event_name = "found_open_space_in_right";
            trigger_event_pub_.publish(event_msg);
        }
        else
        {
            robotx_msgs::Event event_msg;
            event_msg.header.stamp = ros::Time::now();
            event_msg.trigger_event_name = "found_open_space_in_left";
            trigger_event_pub_.publish(event_msg);
        }
        return;
    }
    if(current_state_->current_state == "turn_left")
    {
        geometry_msgs::Twist twist_cmd;
        twist_cmd.angular.z = 0.1;
        twist_cmd_pub_.publish(twist_cmd);
        return;
    }
    if(current_state_->current_state == "turn_right")
    {
        geometry_msgs::Twist twist_cmd;
        twist_cmd.angular.z = -0.1;
        twist_cmd_pub_.publish(twist_cmd);
        return;
    }
    if(obstacle_found_())
    {
        robotx_msgs::Event event_msg;
        event_msg.trigger_event_name = "obstacle_found";
        event_msg.header.stamp = ros::Time::now();
        trigger_event_pub_.publish(event_msg);
        geometry_msgs::Pose2D target_pose_2d;
        geometry_msgs::PoseStamped target_pose;
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(target_pose_->header.frame_id, robot_frame_, ros::Time(0), ros::Duration(1));
            tf2::doTransform(*target_pose_, target_pose, transform_stamped);
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
        target_pose_2d.x = target_pose.pose.position.x;
        target_pose_2d.y = target_pose.pose.position.y;
        tf2::Quaternion quat(target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w);
        double r,p,y;
        tf2::Matrix3x3(quat).getRPY(r, p, y);
        target_pose_2d.theta = y;
        boost::optional<geometry_msgs::Twist> planner_cmd = planner_.plan(map_,odom_,target_pose_2d);
        if(!planner_cmd)
        {
            ROS_ERROR_STREAM("All planed path was failed.");
            geometry_msgs::Twist stop_cmd;
            twist_cmd_pub_.publish(stop_cmd);
            robotx_msgs::Event event_msg;
            event_msg.header.stamp = ros::Time::now();
            event_msg.trigger_event_name = "obstacle_avoid_failed";
            trigger_event_pub_.publish(event_msg);
        }
        else
        {
            ROS_INFO_STREAM("Local path planning succeed.");
            twist_cmd_pub_.publish(*planner_cmd);
        }
    }
    return;
}

void obstacle_avoid::obstacle_map_callback_(const robotx_msgs::ObstacleMap::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    publish_nearest_obstacle_range_();
    map_ = *msg;
    map_recieved_ = true;
    return;
}