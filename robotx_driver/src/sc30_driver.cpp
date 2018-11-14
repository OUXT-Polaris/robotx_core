#include <sc30_driver.h>

sc30_driver::sc30_driver(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("twist_topic", twist_topic_, ros::this_node::getName()+"/twist");
    pnh_.param<std::string>("fix_topic", fix_topic_, ros::this_node::getName()+"/fix");
    navsat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(fix_topic_,10);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_,10);
    nmea_sub_ = nh_.subscribe("/nmea_sentence",10,&sc30_driver::nmea_cakkback_,this);
}

sc30_driver::~sc30_driver()
{
    
}

void sc30_driver::nmea_cakkback_(const nmea_msgs::Sentence::ConstPtr msg)
{
    int type = get_sentence_type_(msg->sentence);
    if(type == SENTENCE_GPRMC)
    {
        boost::optional<sensor_msgs::NavSatFix> fix = parse_gprmc_sentence_(msg);
        if(fix)
        {
            navsat_fix_pub_.publish(*fix);
        }
    }
    return;
}

boost::optional<sensor_msgs::NavSatFix> sc30_driver::parse_gprmc_sentence_(const nmea_msgs::Sentence::ConstPtr sentence)
{
    sensor_msgs::NavSatFix fix;
    std::vector<std::string> splited_sentence = split_(sentence->sentence,',');
    std::string lat_str = splited_sentence[3];
    std::string lon_str = splited_sentence[5];
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
    if(type_str == "$GPVTG")
    {
        return SENTENCE_GPVTG;
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