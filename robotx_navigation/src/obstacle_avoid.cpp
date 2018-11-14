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
    nh_.param<std::string>(ros::this_node::getName()+"/target_pose_topic", target_pose_topic_, ros::this_node::getName()+"/target_pose");
    nh_.param<std::string>(ros::this_node::getName()+"/current_state_topic", current_state_topic_, ros::this_node::getName()+"current_state");
    nh_.param<std::string>(ros::this_node::getName()+"/trigger_event_topic", trigger_event_topic_, ros::this_node::getName()+"/trigger_event");  
    nh_.param<double>(ros::this_node::getName()+"/search_radius", search_radius_, 3.0);
    nh_.param<double>(ros::this_node::getName()+"/search_angle", search_angle_, 0.3);
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    angular_vel_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_angular_vel", 1);
    linear_vel_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_linear_vel", 1);
    trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>(trigger_event_topic_, 1);
    map_sub_ = nh_.subscribe(map_topic_, 3, &obstacle_avoid::obstacle_map_callback_, this);
    twist_cmd_sub_ = nh_.subscribe(raw_cmd_vel_topic_, 10, &obstacle_avoid::twist_cmd_callback_, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &obstacle_avoid::odom_callback_, this);
    target_pose_sub_ = nh_.subscribe(target_pose_topic_, 10, &obstacle_avoid::target_pose_callback_, this);
}

obstacle_avoid::~obstacle_avoid()
{

}

void obstacle_avoid::current_state_callback_(const robotx_msgs::State::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    *current_state_ = *msg;
    return;
}

void obstacle_avoid::target_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_pose_ = *msg;
    return;
}

void obstacle_avoid::twist_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    twist_cmd_recieved_ = true;
    raw_twist_cmd_ = *msg;
    return;
}

bool obstacle_avoid::obstacle_found_(const nav_msgs::Odometry::ConstPtr msg)
{
    if(!target_pose_)
    {
        return false;
    }
    geometry_msgs::TransformStamped target_pose_transform_stamped_;
    try
    {
        target_pose_transform_stamped_ = tf_buffer_.lookupTransform(msg->header.frame_id, target_pose_->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return false;
    }
    tf2::doTransform(*target_pose_, transformed_target_pose_, target_pose_transform_stamped_);
    if(odom_recieved_ && map_recieved_ && twist_cmd_recieved_)
    {
        for (int i = 0; i < map_ptr_->info.height; i++) 
        {
            for (int m = 0; m < map_ptr_->info.width; m++)
            {
                if(map_ptr_->data[i * map_ptr_->info.height + m] > 0)
                {
                    double x = (double)m * map_ptr_->info.resolution - 0.5 * map_ptr_->info.resolution * map_ptr_->info.width;
                    double y = (double)i * map_ptr_->info.resolution - 0.5 * map_ptr_->info.resolution * map_ptr_->info.height;
                    if(x==0 && y==0)
                    {

                    }
                    else
                    {
                        double theta = std::atan2(y,x);
                        double r = std::sqrt(y*y+x*x);
                        if(r<search_radius_ && std::fabs(theta) < search_angle_)
                        {
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

void obstacle_avoid::odom_callback_(const nav_msgs::Odometry::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_recieved_ = true;
    odom_ = *msg;
    geometry_msgs::Twist twist_cmd;
    if(obstacle_found_(msg))
    {
        robotx_msgs::Event event_msg;
        event_msg.trigger_event_name = "obstacle_found";
        event_msg.header.stamp = ros::Time::now();
        trigger_event_pub_.publish(event_msg);
        double x = transformed_target_pose_.pose.position.x;
        double y = transformed_target_pose_.pose.position.y;
        double theta = std::atan2(y,x);
        if(theta > 0)
        {
            twist_cmd.linear.x = 0;
            twist_cmd.angular.z = 0.3;
        }
        else
        {
            twist_cmd.linear.x = 0;
            twist_cmd.angular.z = -0.3;                
        }
    }
    twist_cmd_pub_.publish(twist_cmd);
    std_msgs::Float32 linear_vel_msg,angular_vel_msg;
    linear_vel_msg.data = twist_cmd.linear.x;
    angular_vel_msg.data = twist_cmd.angular.z;
    linear_vel_pub_.publish(linear_vel_msg);
    angular_vel_pub_.publish(angular_vel_msg);
    return;
}

void obstacle_avoid::obstacle_map_callback_(const nav_msgs::OccupancyGrid::Ptr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    map_ptr_ = boost::shared_ptr<nav_msgs::OccupancyGrid>(msg);
    map_recieved_ = true;
    return;
}