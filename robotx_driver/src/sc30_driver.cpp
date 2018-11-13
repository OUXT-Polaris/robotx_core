#include <sc30_driver.h>

sc30_driver::sc30_driver(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    navsat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fix",10);
    nmea_sub_ = nh_.subscribe("/nmea_sentence",10,&sc30_driver::nmea_cakkback_,this);
}

sc30_driver::~sc30_driver()
{
    
}

void sc30_driver::nmea_cakkback_(const nmea_msgs::Sentence::ConstPtr msg)
{
    return;
}