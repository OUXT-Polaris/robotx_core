#ifndef COAST_LINE_PUBLISHER_H_INCLUDED
#define COAST_LINE_PUBLISHER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include <UTM.h>
#include <robotx_msgs/GeographicMap.h>
#include <robotx_msgs/CoastLineArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

//headers in STL
#include <mutex>

//headers in boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

struct point_with_area
{
    int area;
    uint64_t node_id;
    geometry_msgs::Point point;
};

class coast_line_publisher
{
public:
    coast_line_publisher(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~coast_line_publisher();
private:
    std::mutex mtx_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    double range_;
    std::string world_frame_;
    std::string fix_topic_;
    std::string geographic_map_topic_;
    ros::Subscriber fix_sub_;
    ros::Subscriber geographic_map_sub_;
    ros::Publisher coast_line_pub_;
    ros::Publisher marker_pub_;
    robotx_msgs::CoastLineArray get_coast_lines_();
    robotx_msgs::CoastLineArray current_coast_lines_;
    boost::optional<point_with_area> query_point_(int node_id, std::vector<point_with_area> points_in_area);
    std::vector<point_with_area> all_points_;
    std::vector<robotx_msgs::GeographicLine> all_lines_;
    std::vector<point_with_area> filter_points_();
    boost::circular_buffer<int> utm_area_buf_;
    void fix_callback_(const sensor_msgs::NavSatFixConstPtr msg);
    void map_callback_(const robotx_msgs::GeographicMapConstPtr msg);
    point_with_area convert_geopoint_(robotx_msgs::GeographicPoint geopoint);
    void publish_marker_();
};

#endif  //COAST_LINE_PUBLISHER_H_INCLUDED