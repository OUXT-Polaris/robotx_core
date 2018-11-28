#include <coast_line_publisher.h>

coast_line_publisher::coast_line_publisher(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<double>("range", range_, 0.0);
    pnh_.param<bool>("enable_publish_marker", enable_publish_marker_, true);
    pnh_.param<std::string>("coast_line_csv_filename", coast_line_csv_filename_,"/coastline.csv");
    pnh_.param<std::string>("world_frame", world_frame_,"world");
    coast_line_pub_ = pnh_.advertise<robotx_msgs::CoastLineArray>("coast_lines",1,true);
    if(enable_publish_marker_ == true)
    {
        marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
    }
    coast_line_csv_filepath_ = ros::package::getPath("robotx_navigation")+"/data/"+coast_line_csv_filename_;
    std::ifstream ifs(coast_line_csv_filepath_.c_str());
    std::string line;
    current_coast_lines_.header.frame_id = world_frame_;
    while (getline(ifs, line))
    {
        std::vector<std::string> strvec = split_(line, ',');
        robotx_msgs::CoastLine coast_line;
        coast_line.header.frame_id = world_frame_;
        try
        {
            coast_line.start_point.x = std::stod(strvec[0].c_str());
            coast_line.start_point.y = std::stod(strvec[1].c_str());
            coast_line.start_point.z = 0;
            coast_line.end_point.x = std::stod(strvec[4].c_str());
            coast_line.end_point.y = std::stod(strvec[5].c_str());
            coast_line.end_point.z = 0;
        }
        catch(...)
        {
            ROS_ERROR_STREAM("failed to load coastline.");
            std::exit(-1);
        }
        current_coast_lines_.coast_lines.push_back(coast_line);
    }
}

coast_line_publisher::~coast_line_publisher()
{

}

void coast_line_publisher::run()
{
    coast_line_pub_.publish(current_coast_lines_);
    if(enable_publish_marker_ == true)
    {
        generate_marker_();
    }
    ros::Rate rate(1);
    while(ros::ok())
    {
        if(enable_publish_marker_ == true)
        {
            publish_marker_();
        }
        rate.sleep();
    }
}

std::vector<std::string> coast_line_publisher::split_(std::string& input, char delimiter)
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

void coast_line_publisher::publish_marker_()
{
    ros::Time now = ros::Time::now();
    for(auto marker_itr = marker_.markers.begin(); marker_itr != marker_.markers.end(); marker_itr++)
    {
        marker_itr->header.stamp = now;
    }
    marker_pub_.publish(marker_);
    return;
}

void coast_line_publisher::generate_marker_()
{
    marker_.markers.clear();
   int current_id = 0;
    for(auto coast_line_itr = current_coast_lines_.coast_lines.begin(); coast_line_itr != current_coast_lines_.coast_lines.end(); coast_line_itr++)
    {
        visualization_msgs::Marker single_marker;
        single_marker.header.frame_id = world_frame_;
        single_marker.type = visualization_msgs::Marker::CYLINDER;
        single_marker.action = visualization_msgs::Marker::ADD;
        single_marker.frame_locked = true;
        single_marker.ns = "coast_line";
        single_marker.id = current_id;
        single_marker.pose.position.x = (coast_line_itr->start_point.x + coast_line_itr->end_point.x)/2;
        single_marker.pose.position.y = (coast_line_itr->start_point.y + coast_line_itr->end_point.y)/2;
        single_marker.pose.position.z = 0;
        double yaw = std::atan2(coast_line_itr->start_point.y - coast_line_itr->end_point.y, 
            coast_line_itr->start_point.x - coast_line_itr->end_point.x);
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,M_PI/2,yaw);
        quaternionTFToMsg(quaternion, single_marker.pose.orientation);
        single_marker.scale.x = 1.0;
        single_marker.scale.y = 1.0;
        single_marker.scale.z = std::sqrt(std::pow(coast_line_itr->start_point.y-coast_line_itr->end_point.y,2)
            +std::pow(coast_line_itr->start_point.x-coast_line_itr->end_point.x,2));
        single_marker.color.r = 1;
        single_marker.color.g = 0;
        single_marker.color.b = 0;
        single_marker.color.a = 1;
        marker_.markers.push_back(single_marker);
        current_id++;
    }
    return;
}