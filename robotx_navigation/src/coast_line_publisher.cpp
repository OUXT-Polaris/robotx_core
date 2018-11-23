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
        marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("marker",1);
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
    marker_.header.stamp = ros::Time::now();
    marker_pub_.publish(marker_);
    return;
}

void coast_line_publisher::generate_marker_()
{
    marker_.type = visualization_msgs::Marker::LINE_LIST;
    marker_.header.frame_id = world_frame_;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.frame_locked = true;
    marker_.ns = "coast_line";
    marker_.scale.x = 1;
    for(auto coast_line_itr = current_coast_lines_.coast_lines.begin(); coast_line_itr != current_coast_lines_.coast_lines.end(); coast_line_itr++)
    {
        marker_.points.push_back(coast_line_itr->start_point);
        marker_.points.push_back(coast_line_itr->end_point);
        std_msgs::ColorRGBA color;
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 1;
        marker_.colors.push_back(color);
        marker_.colors.push_back(color);
    }
    return;
}