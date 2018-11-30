#include <field_map_server.h>

field_map_server::field_map_server(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    field_map_pub_ = nh_.advertise<robotx_msgs::FieldMap>("/field_map",1,true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/field_map/marker",1,true);
}

field_map_server::~field_map_server()
{

}

void field_map_server::run()
{
    load_();
    field_map_pub_.publish(field_map_);
    marker_pub_.publish(markers_);
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
    int i = 0;
    while (getline(green_ifs, green_line))
    {
        std::vector<std::string> strvec = split_(green_line, ',');
        geometry_msgs::Point p;
        p.x = std::stod(strvec[0]);
        p.y = std::stod(strvec[1]);
        p.z = std::stod(strvec[2]);
        field_map_.green_buoys.push_back(p);
        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = map_frame_;
        marker_msg.ns = "green_buoy";
        marker_msg.action = marker_msg.ADD;
        marker_msg.type = marker_msg.MESH_RESOURCE;
        marker_msg.id = i;
        marker_msg.pose.position = p;
        marker_msg.pose.orientation.x = 0;
        marker_msg.pose.orientation.y = 0;
        marker_msg.pose.orientation.z = 0;
        marker_msg.pose.orientation.w = 1;
        marker_msg.scale.x = 1;
        marker_msg.scale.y = 1;
        marker_msg.scale.z = 1;
        marker_msg.color.r = 0;
        marker_msg.color.g = 1;
        marker_msg.color.b = 0;
        marker_msg.color.a = 1;
        marker_msg.frame_locked = true;
        marker_msg.mesh_resource = "package://robotx_gazebo/models/surmark950400/mesh/surmark950400.dae";
        marker_msg.mesh_use_embedded_materials = true;
        markers_.markers.push_back(marker_msg);
        i++;
    }
    std::ifstream red_ifs(red_buoy_csv.c_str());
    std::string red_line;
    i = 0;
    while (getline(red_ifs, red_line))
    {
        std::vector<std::string> strvec = split_(red_line, ',');
        geometry_msgs::Point p;
        p.x = std::stod(strvec[0]);
        p.y = std::stod(strvec[1]);
        p.z = std::stod(strvec[2]);
        field_map_.red_buoys.push_back(p);
        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = map_frame_;
        marker_msg.ns = "red_buoy";
        marker_msg.action = marker_msg.ADD;
        marker_msg.type = marker_msg.MESH_RESOURCE;
        marker_msg.id = i;
        marker_msg.pose.position = p;
        marker_msg.pose.orientation.x = 0;
        marker_msg.pose.orientation.y = 0;
        marker_msg.pose.orientation.z = 0;
        marker_msg.pose.orientation.w = 1;
        marker_msg.scale.x = 1;
        marker_msg.scale.y = 1;
        marker_msg.scale.z = 1;
        marker_msg.color.r = 1;
        marker_msg.color.g = 0;
        marker_msg.color.b = 0;
        marker_msg.color.a = 1;
        marker_msg.frame_locked = true;
        marker_msg.mesh_resource = "package://robotx_gazebo/models/surmark950410/mesh/surmark950410.dae";
        marker_msg.mesh_use_embedded_materials = true;
        markers_.markers.push_back(marker_msg);
        i++;
    }
    std::ifstream white_ifs(white_buoy_csv.c_str());
    std::string white_line;
    i = 0;
    while (getline(white_ifs, white_line))
    {
        std::vector<std::string> strvec = split_(white_line, ',');
        geometry_msgs::Point p;
        p.x = std::stod(strvec[0]);
        p.y = std::stod(strvec[1]);
        p.z = std::stod(strvec[2]);
        field_map_.white_buoys.push_back(p);
        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = map_frame_;
        marker_msg.ns = "white_buoy";
        marker_msg.action = marker_msg.ADD;
        marker_msg.type = marker_msg.MESH_RESOURCE;
        marker_msg.id = i;
        marker_msg.pose.position = p;
        marker_msg.pose.orientation.x = 0;
        marker_msg.pose.orientation.y = 0;
        marker_msg.pose.orientation.z = 0;
        marker_msg.pose.orientation.w = 1;
        marker_msg.scale.x = 1;
        marker_msg.scale.y = 1;
        marker_msg.scale.z = 1;
        marker_msg.color.r = 1;
        marker_msg.color.g = 1;
        marker_msg.color.b = 1;
        marker_msg.color.a = 1;
        marker_msg.frame_locked = true;
        marker_msg.mesh_resource = "package://robotx_gazebo/models/surmark46104/mesh/surmark46104.dae";
        marker_msg.mesh_use_embedded_materials = true;
        markers_.markers.push_back(marker_msg);
        i++;
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