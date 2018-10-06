#include <waypoint_server.h>

waypoint_server::waypoint_server()
{
    std::string waypoint_bag_filename;
    nh_.param<std::string>(ros::this_node::getName()+"/waypoint_bag_filename", waypoint_bag_filename, "waypoints.bag");
    waypoint_bag_file_path_ = ros::package::getPath("robotx_navigation") + "/data/" + waypoint_bag_filename;
    rosbag::Bag bag;
    bag.open(waypoint_bag_file_path_);
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        robotx_msgs::WayPointArray::ConstPtr i = m.instantiate<robotx_msgs::WayPointArray>();
        if(i != NULL)
        {
            waypoints_ = *i;
            break;
        }
    }
    bag.close();
}

waypoint_server::~waypoint_server()
{

}