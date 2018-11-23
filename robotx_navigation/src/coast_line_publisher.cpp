#include <coast_line_publisher.h>

coast_line_publisher::coast_line_publisher(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<double>("range", range_, 0.0);
    pnh_.param<std::string>("coast_line_csv_filename", coast_line_csv_filename_,"/coastline.csv");
    coast_line_pub_ = pnh_.advertise<robotx_msgs::CoastLineArray>("coast_lines",10);
    marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("marker",10);
    coast_line_csv_filepath_ = ros::package::getPath("robotx_navigation")+"/data/"+coast_line_csv_filename_;
    std::ifstream ifs(coast_line_csv_filepath_.c_str());
    std::string line;
    while (getline(ifs, line))
    {
        std::vector<std::string> strvec = split(line, ',');
    }
}

coast_line_publisher::~coast_line_publisher()
{

}

std::vector<std::string> coast_line_publisher::split(std::string& input, char delimiter)
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
    marker_.type = marker_.LINE_LIST;
    marker_.header.frame_id = world_frame_;
    marker_.action = marker_.ADD;
    marker_.ns = "coast_line";
    marker_.points.clear();
    marker_.colors.clear();
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
    }
    return;
}