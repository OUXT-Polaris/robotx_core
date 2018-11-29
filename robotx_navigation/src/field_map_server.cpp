#include <field_map_server.h>

field_map_server::field_map_server(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    field_map_pub_ = nh_.advertise<robotx_msgs::FieldMap>("/field_map",1,true);
}

field_map_server::~field_map_server()
{

}

void field_map_server::run()
{
    load_();
    field_map_pub_.publish(field_map_);
    return;
}

void field_map_server::load_()
{
    std::string green_buoy_csv = ros::package::getPath("robotx_navigation") + "/data/green_buoys.csv";
    std::string red_buoy_csv = ros::package::getPath("robotx_navigation") + "/data/red_buoys.csv";
    std::string white_buoy_csv = ros::package::getPath("robotx_navigation") + "/data/white_buoys.csv";
    field_map_.header.frame_id = map_frame_;
    std::ifstream green_ifs(green_buoy_csv.c_str());
    std::string green_line;
    while (getline(green_ifs, green_line))
    {
        std::vector<std::string> strvec = split_(green_line, ',');
        geometry_msgs::Point p;
        p.x = std::stod(strvec[0]);
        p.y = std::stod(strvec[1]);
        p.z = std::stod(strvec[2]);
        field_map_.green_buoys.push_back(p);
    }
    std::ifstream red_ifs(red_buoy_csv.c_str());
    std::string red_line;
    while (getline(red_ifs, red_line))
    {
        std::vector<std::string> strvec = split_(red_line, ',');
        geometry_msgs::Point p;
        p.x = std::stod(strvec[0]);
        p.y = std::stod(strvec[1]);
        p.z = std::stod(strvec[2]);
        field_map_.red_buoys.push_back(p);
    }
    std::ifstream white_ifs(white_buoy_csv.c_str());
    std::string white_line;
    while (getline(white_ifs, white_line))
    {
        std::vector<std::string> strvec = split_(white_line, ',');
        geometry_msgs::Point p;
        p.x = std::stod(strvec[0]);
        p.y = std::stod(strvec[1]);
        p.z = std::stod(strvec[2]);
        field_map_.white_buoys.push_back(p);
    }
    return;
}

std::vector<std::string> field_map_server::split_(std::string& input, char delimiter)
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