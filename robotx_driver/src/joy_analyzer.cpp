#include <joy_analyzer.h>

joy_analyzer::joy_analyzer()
{
    buf_ = boost::circular_buffer<sensor_msgs::Joy>(2);
}

joy_analyzer::~joy_analyzer()
{

}

void joy_analyzer::add_data(sensor_msgs::Joy joy)
{
    buf_.push_back(joy);
    return;
}

bool joy_analyzer::button_pressed(int index)
{
    if(buf_.size() == 1)
    {
        if(buf_[0].buttons[index]  == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    if(buf_[0].buttons[index]  == 0 && buf_[1].buttons[index] == 1)
    {
        return true;
    }
    return false;
}

bool joy_analyzer::button_released(int index)
{
    if(buf_.size() == 1)
    {
        if(buf_[0].buttons[index]  == 1)
        {
            return false;
        }
        /*
        else
        {
            return true;
        }
        */
    }
    if(buf_[0].buttons[index]  == 1 && buf_[1].buttons[index] == 0)
    {
        return true;
    }
    return false;
}