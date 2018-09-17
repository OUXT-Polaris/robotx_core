//headers in boost
#include <boost/optional.hpp>

//headers in STL
#include <mutex>

//headers in ROS
#include <geometry_msgs/PoseStamped.h>

class robot_state
{
public:
    robot_state() : _current_pose(boost::none), _goal_pose(boost::none)
    {

    }
    ~robot_state()
    {

    }
    void update_current_pose(geometry_msgs::PoseStamped pose)
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _current_pose = pose;
    }
    void update_goal_pose(geometry_msgs::PoseStamped pose)
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _goal_pose = pose;
    }
private:
    std::mutex _mtx;
    boost::optional<geometry_msgs::PoseStamped> _current_pose;
    boost::optional<geometry_msgs::PoseStamped> _goal_pose;
};