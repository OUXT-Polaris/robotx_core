//headers in boost
#include <boost/optional.hpp>

//headers in STL
#include <mutex>

//headers in ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

struct robot_state_info
{
    const boost::optional<geometry_msgs::TwistStamped> current_twist;
    const boost::optional<geometry_msgs::PoseStamped> current_pose;
    const boost::optional<geometry_msgs::PoseStamped> goal_pose;
    robot_state_info(boost::optional<geometry_msgs::TwistStamped> _current_twist, 
        boost::optional<geometry_msgs::PoseStamped> _current_pose,
        boost::optional<geometry_msgs::PoseStamped> _goal_pose)
        : current_twist(_current_twist), current_pose(_current_pose), goal_pose(_goal_pose)
    {

    }
};

class robot_state
{
public:
    robot_state() : _current_pose(boost::none), _goal_pose(boost::none), _current_twist(boost::none)
    {

    }
    ~robot_state()
    {

    }
    void update_current_state(geometry_msgs::PoseStamped pose, geometry_msgs::TwistStamped twist)
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _current_pose = pose;
        _current_twist = twist;
    }
    void update_goal_pose(geometry_msgs::PoseStamped pose)
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _goal_pose = pose;
    }
    robot_state_info get_robot_state()
    {
        robot_state_info ret(_current_twist,_current_pose,_goal_pose);
        return ret;
    }
private:
    std::mutex _mtx;
    boost::optional<geometry_msgs::TwistStamped> _current_twist;
    boost::optional<geometry_msgs::PoseStamped> _current_pose;
    boost::optional<geometry_msgs::PoseStamped> _goal_pose;
};