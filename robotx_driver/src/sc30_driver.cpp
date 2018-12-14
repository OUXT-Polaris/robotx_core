#include <sc30_driver.h>

sc30_driver::sc30_driver(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    true_course_buf_ = boost::circular_buffer<std::pair<ros::Time,double> >(2);
    utm_point_buf_ = boost::circular_buffer<geodesy::UTMPoint>(2);
    pnh_.param<std::string>("twist_topic", twist_topic_, ros::this_node::getName()+"/twist");
    pnh_.param<std::string>("fix_topic", fix_topic_, ros::this_node::getName()+"/fix");
    pnh_.param<std::string>("true_course_topic", true_course_topic_, ros::this_node::getName()+"/true_course");
    navsat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(fix_topic_,10);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_topic_,10);
    true_course_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(true_course_topic_,10);
    callback_func_type_ = boost::bind(&sc30_driver::configure_callback_, this, _1, _2);
    server_.setCallback(callback_func_type_);
    nmea_sub_ = nh_.subscribe("/nmea_sentence",10,&sc30_driver::nmea_cakkback_,this);
}

sc30_driver::~sc30_driver()
{
    
}

double sc30_driver::get_diff_angle_(double from,double to)
{
    double ans = 0;
    double inner_prod = std::cos(from)*std::cos(to)+std::sin(from)*std::sin(to);
    double theta = std::acos(inner_prod);
    double outer_prod = std::cos(from)*std::sin(to)-std::sin(from)*std::cos(to);
    if(outer_prod > 0)
    {
        ans = theta;
    }
    else
    {
        ans = -1 * theta;
    }
    return ans;
}

void sc30_driver::nmea_cakkback_(const nmea_msgs::Sentence::ConstPtr msg)
{
    int type = get_sentence_type_(msg->sentence);
    if(type == SENTENCE_GPRMC)
    {
        boost::optional<sensor_msgs::NavSatFix> fix = get_nav_sat_fix_(msg);
        boost::optional<geometry_msgs::TwistStamped> twist = get_twist_(msg);
        //bool status = is_valid_status_(msg);
        if(fix)// && status)
        {
            navsat_fix_pub_.publish(*fix);
        }
        if(twist)// && status)
        {
            twist_pub_.publish(*twist);
        }
    }
    if(type == SENTENCE_GPHDT)
    {
        boost::optional<geometry_msgs::QuaternionStamped> quat = get_true_course_(msg);
        if(quat)
        {
            true_course_pub_.publish(*quat);
        }
    }
    return;
}

void sc30_driver::configure_callback_(robotx_driver::sc30_driverConfig &config, uint32_t level)
{
    offset_angle_ = config.offset_angle;
    true_course_buf_.clear();
    return;
}

bool sc30_driver::is_valid_status_(const nmea_msgs::Sentence::ConstPtr sentence)
{
    std::vector<std::string> splited_sentence = split_(sentence->sentence,',');
    if(splited_sentence[2]  == "A")
    {
        return true;
    }
    ROS_ERROR_STREAM("SC30 status is not valid.");
    return false;
}

boost::optional<geometry_msgs::QuaternionStamped> sc30_driver::get_true_course_(const nmea_msgs::Sentence::ConstPtr sentence)
{
    geometry_msgs::QuaternionStamped true_course;
    // get true course from GPHDT
    std::vector<std::string> splited_sentence = split_(sentence->sentence,',');
    std::string true_course_str = splited_sentence[1];
    try
    {
        double true_course_value = std::stod(true_course_str);
        std::pair<ros::Time,double> data;
        data.first = sentence->header.stamp;
        data.second = -1*(true_course_value/180*M_PI);
        true_course_buf_.push_back(data);
        true_course.header = sentence->header;
        tf::Quaternion quat;
        quat.setRPY(0,0,data.second);
        quaternionTFToMsg(quat, true_course.quaternion);
    }
    catch(...)
    {
        return boost::none;
    }
    return true_course;
}

boost::optional<geometry_msgs::TwistStamped> sc30_driver::get_twist_(const nmea_msgs::Sentence::ConstPtr sentence)
{
    geometry_msgs::TwistStamped twist;
    std::vector<std::string> splited_sentence = split_(sentence->sentence,',');
    if(true_course_buf_.size() == 0 && utm_point_buf_.size() != 2)
    {
        return boost::none;
    }
    std::string speed_str = splited_sentence[7];
    std::string twist_dir_str = splited_sentence[8];
    try
    {
        double twist_dir = std::stod(twist_dir_str)/180*M_PI;
        double true_course = true_course_buf_[true_course_buf_.size()-1].second;
        double diff_angle_ = get_diff_angle_(true_course,twist_dir);
        double trans = std::sqrt(std::pow(utm_point_buf_[1].easting-utm_point_buf_[0].easting,2) + std::pow(utm_point_buf_[1].northing-utm_point_buf_[0].northing,2));
        double speed_val = std::stod(speed_str);
        if(speed_val > trans)
        {
            speed_val = trans;
        }
        double speed = 0;
        if(std::fabs(diff_angle_) > (M_PI*0.5))
        {
            speed = -1 * speed_val;
        }
        else
        {
            speed = speed_val;
        }
        twist.header = sentence->header;
        twist.twist.linear.x = speed; 
        twist.twist.linear.y = 0;
        twist.twist.linear.z = 0;
        twist.twist.angular.x = 0;
        twist.twist.angular.y = 0;
        if(true_course_buf_.size() != 2)
        {
            twist.twist.angular.z = 0;
        }
        else
        {
            twist.twist.angular.z = get_diff_angle_(true_course_buf_[1].second,true_course_buf_[0].second)/(true_course_buf_[1].first,true_course_buf_[0].first).toSec();
        }
    }
    catch(...)
    {
        return boost::none;
    }
    return twist;
}

boost::optional<sensor_msgs::NavSatFix> sc30_driver::get_nav_sat_fix_(const nmea_msgs::Sentence::ConstPtr sentence)
{
    sensor_msgs::NavSatFix fix;
    std::vector<std::string> splited_sentence = split_(sentence->sentence,',');
    std::string lat_str = splited_sentence[3];
    std::string north_or_south_str = splited_sentence[4];
    std::string lon_str = splited_sentence[5];
    std::string east_or_west_str = splited_sentence[6];
    try
    {
        std::string lat_min_str = lat_str.substr(lat_str.length()-(size_t)7,(size_t)7);
        std::string lon_min_str = lon_str.substr(lon_str.length()-(size_t)7,(size_t)7);
        std::string lat_deg_str = lat_str.substr((size_t)0,lat_str.length()-lat_min_str.length());
        std::string lon_deg_str = lon_str.substr((size_t)0,lon_str.length()-lon_min_str.length());
        double lat_min = std::stod(lat_min_str);
        double lon_min = std::stod(lon_min_str);
        double lat_deg = std::stod(lat_deg_str);
        double lon_deg = std::stod(lon_deg_str);
        fix.latitude = lat_deg + lat_min/60.0;
        fix.longitude = lon_deg + lon_min/60.0;
        if(north_or_south_str == "S")
        {
            fix.latitude = fix.latitude * -1;
        }
        if(east_or_west_str == "W")
        {
            fix.longitude = fix.longitude * -1;
        }
        geographic_msgs::GeoPoint geo_poinst;
        geo_poinst.latitude = fix.latitude;
        geo_poinst.longitude = fix.longitude;
        geodesy::UTMPoint utm_point(geo_poinst);
        utm_point_buf_.push_back(utm_point);
        fix.header = sentence->header;
    }
    catch(...)
    {
        return boost::none;
    }
    return fix;
}

int sc30_driver::get_sentence_type_(std::string sentence)
{
    std::string type_str = split_(sentence,',')[0];
    if(type_str == "$GPRMC")
    {
        return SENTENCE_GPRMC;
    }
    if(type_str == "$GPHDT")
    {
        return SENTENCE_GPHDT;
    }
    return SENTENCE_UNDEFINED;
}

std::vector<std::string> sc30_driver::split_(const std::string &s,char delim)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim))
    {
        if (!item.empty())
        {
            elems.push_back(item);
        }
    }
    return elems;
}