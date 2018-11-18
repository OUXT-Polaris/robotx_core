#ifndef JOY_ANALYZER_H_INCLUDED
#define JOY_ANALYZER_H_INCLUDED

//headers in boost
#include <boost/circular_buffer.hpp>

//headers in ROS
#include <sensor_msgs/Joy.h>

class joy_analyzer
{
public:
    joy_analyzer();
    ~joy_analyzer();
    void add_data(sensor_msgs::Joy joy);
    bool button_pressed(int index);
    bool button_released(int index);
private:
    boost::circular_buffer<sensor_msgs::Joy> buf_;
};

#endif  //JOY_ANALYZER_H_INCLUDED