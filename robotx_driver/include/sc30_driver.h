#ifndef SC30_DRIVER_H_INCLUDED
#define SC30_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/tf.h>

//headers in STL
#include <vector>
#include <string>
#include <sstream>

//headers in boost
#include <boost/optional.hpp>
#include <boost/circular_buffer.hpp>

#define SENTENCE_UNDEFINED 0
#define SENTENCE_GPRMC 1
#define SENTENCE_GPHDT 2

class sc30_driver
{
public:
    sc30_driver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~sc30_driver();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void nmea_cakkback_(const nmea_msgs::Sentence::ConstPtr msg);
    int get_sentence_type_(std::string sentence);
    boost::optional<sensor_msgs::NavSatFix> get_nav_sat_fix_(const nmea_msgs::Sentence::ConstPtr sentence);
    boost::optional<geometry_msgs::QuaternionStamped> get_true_course_(const nmea_msgs::Sentence::ConstPtr sentence);
    boost::optional<geometry_msgs::TwistStamped> get_twist_(const nmea_msgs::Sentence::ConstPtr sentence);
    bool is_valid_status_(const nmea_msgs::Sentence::ConstPtr sentence);
    double get_diff_angle_(double from,double to);
    std::vector<std::string> split_(const std::string &s, char delim);
    ros::Subscriber nmea_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher navsat_fix_pub_;
    ros::Publisher true_course_pub_;
    std::string twist_topic_;
    std::string fix_topic_;
    std::string true_course_topic_;
    boost::circular_buffer<std::pair<ros::Time,double> > true_course_buf_;
};

#endif  //SC30_DRIVER_H_INCLUDED