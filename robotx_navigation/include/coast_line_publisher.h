#ifndef COAST_LINE_PUBLISHER_H_INCLUDED
#define COAST_LINE_PUBLISHER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>

//headers in this package
#include <UTM.h>
#include <robotx_msgs/CoastLineArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

//headers in STL
#include <mutex>
#include <fstream>
#include <sstream>

//headers in boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

class coast_line_publisher
{
public:
    coast_line_publisher(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~coast_line_publisher();
    void run();
private:
    std::vector<std::string> split_(std::string& input, char delimiter);
    void generate_marker_();
    void publish_marker_();
    std::mutex mtx_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    double range_;
    std::string world_frame_;
    std::string fix_topic_;
    std::string coast_line_csv_filename_;
    std::string coast_line_csv_filepath_;
    bool enable_publish_marker_;
    ros::Publisher coast_line_pub_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;
    robotx_msgs::CoastLineArray current_coast_lines_;
};

#endif  //COAST_LINE_PUBLISHER_H_INCLUDED