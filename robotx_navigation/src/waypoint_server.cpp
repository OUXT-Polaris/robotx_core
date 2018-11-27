#include <waypoint_server.h>

waypoint_server::waypoint_server() : tf_listener_(tf_buffer_)
{
    first_waypoint_finded_ = false;
    std::string waypoint_csv_filename;
    nh_.param<std::string>(ros::this_node::getName()+"/waypoint_csv_filename", waypoint_csv_filename, "waypoints.csv");
    nh_.param<std::string>(ros::this_node::getName()+"/robot_frame", robot_frame_, "base_link");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "map");
    nh_.param<std::string>(ros::this_node::getName()+"/robot_pose_topic", robot_pose_topic_, ros::this_node::getName()+"/robot_pose");
    nh_.param<double>(ros::this_node::getName()+"/search_angle", search_angle_, 1.57);
    waypoint_csv_file_path_ = ros::package::getPath("robotx_navigation") + "/data/" + waypoint_csv_filename;
    std::ifstream ifs(waypoint_csv_file_path_.c_str());
    std::string line;
    while (getline(ifs, line))
    {
        std::vector<std::string> strvec = split_(line, ',');
        geometry_msgs::PoseStamped pose;
        try
        {
            pose.header.frame_id = strvec[0];
            pose.pose.position.x = std::stod(strvec[1].c_str());
            pose.pose.position.y = std::stod(strvec[2].c_str());
            pose.pose.position.z = 0;
            double yaw = std::stod(strvec[3].c_str());
            tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,yaw);
            tf::quaternionTFToMsg(quat,pose.pose.orientation);
        }
        catch(...)
        {
            ROS_ERROR_STREAM("failed to load coastline.");
            std::exit(-1);
        }
        waypoints_.push_back(pose);
    }
    trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>("/robotx_state_machine_node/navigation_state_machine/trigger_event",1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/marker",1);
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(ros::this_node::getName()+"/next_waypoint",1,true);
    robot_pose_sub_ = nh_.subscribe(robot_pose_topic_, 1, &waypoint_server::robot_pose_callback_, this);
    navigation_status_sub_ = nh_.subscribe("/robotx_state_machine_node/navigation_state_machine/current_state",
        1, &waypoint_server::navigation_status_callback_, this);
    boost::thread marker_thread(boost::bind(&waypoint_server::publish_marker_, this));
}

waypoint_server::~waypoint_server()
{

}

std::vector<std::string> waypoint_server::split_(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter))
    {
        result.push_back(field);
    }
    return result;
}

void waypoint_server::publish_marker_()
{
    while(ros::ok())
    {
        visualization_msgs::MarkerArray marker_msg;
        ros::Rate rate(10);
        mutex_.lock();
        for(int i=0; i<waypoints_.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = waypoints_[i].header.frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoint";
            marker.id = i;
            marker.type = marker.ARROW;
            marker.action = marker.MODIFY;
            marker.pose = waypoints_[i].pose;
            if(first_waypoint_finded_ && target_waypoint_index_ == i)
            {
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
            }
            else
            {
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
            }
            marker.color.a = 1;
            marker.scale.x = 1;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.frame_locked = true;
            //marker.lifetime = ros::Duration(0.3);
            marker_msg.markers.push_back(marker);
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = waypoints_[i].header.frame_id;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "id";
            text_marker.id = i;
            text_marker.type = text_marker.TEXT_VIEW_FACING;
            text_marker.action = text_marker.MODIFY;
            text_marker.pose = waypoints_[i].pose;
            text_marker.pose.position.z = text_marker.pose.position.z + 0.3;
            text_marker.pose.orientation.x = 0;
            text_marker.pose.orientation.y = 0;
            text_marker.pose.orientation.z = 0;
            text_marker.pose.orientation.w = 1;
            text_marker.color.r = 1;
            text_marker.color.g = 1;
            text_marker.color.b = 1;
            text_marker.color.a = 1;
            text_marker.scale.x = 1;
            text_marker.scale.y = 1;
            text_marker.scale.z = 1;
            text_marker.frame_locked = true;
            text_marker.text = std::to_string(i);
            //text_marker.lifetime = ros::Duration(0.3);
            marker_msg.markers.push_back(text_marker);
            marker_pub_.publish(marker_msg);
            rate.sleep();
        }
        mutex_.unlock();
    }
    return;
}

void waypoint_server::navigation_status_callback_(robotx_msgs::State msg)
{
    robotx_msgs::Event event_msg;
    event_msg.header.stamp = ros::Time::now();
    if(msg.current_state == "load_next_waypoint")
    {
        if(load_next_waypoint_())
        {
            event_msg.trigger_event_name = "next_waypoint_found";
            trigger_event_pub_.publish(event_msg);
            waypoint_pub_.publish(waypoints_[target_waypoint_index_]);
        }
        else
        {
            event_msg.trigger_event_name = "next_waypoint_not_found";
            trigger_event_pub_.publish(event_msg);
        }
    }
    return;
}

bool waypoint_server::load_next_waypoint_()
{
    if(target_waypoint_index_ != (waypoints_.size()-1))
    {
        target_waypoint_index_++;
        return true;
    }
    return false;
}

void waypoint_server::robot_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    if(first_waypoint_finded_ == false)
    {
        boost::optional<int> nearest_wayppoint_index = get_nearest_wayppoint_(msg);
        if(nearest_wayppoint_index)
        {
            target_waypoint_index_ = nearest_wayppoint_index.get();
            first_waypoint_finded_ = true;
        }
    }
    return;
}

boost::optional<int> waypoint_server::get_nearest_wayppoint_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    boost::optional<int> nearest_wayppoint_index;

    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(robot_frame_, map_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return boost::none;
    }
    int num_waypoints = waypoints_.size();
    geometry_msgs::PoseStamped transformed_pose;
    //std::vector<double> directions;
    std::vector<double> ranges;

    std::vector<int> filtered_index;
    for(int i=0; i<num_waypoints; i++)
    {
        tf2::doTransform(waypoints_[i], transformed_pose, transform_stamped);
        geometry_msgs::Quaternion q = transformed_pose.pose.orientation;
        double r,p,y;
        tf2::Quaternion quat(q.x,q.y,q.z,q.w);
        tf2::Matrix3x3(quat).getRPY(r,p,y);
        if(std::fabs(y) < (search_angle_/2))
        {
            double r = std::sqrt(std::pow(transformed_pose.pose.position.x,2)+std::pow(transformed_pose.pose.position.y,2));
            ranges.push_back(r);
            filtered_index.push_back(i);
        }
    }
    if(ranges.size() == 0)
    {
        return boost::none;
    }
    std::vector<double>::iterator min_iter = std::min_element(ranges.begin(), ranges.end());
    int index = std::distance(ranges.begin(), min_iter);
    nearest_wayppoint_index = filtered_index[index];
    return nearest_wayppoint_index;
}