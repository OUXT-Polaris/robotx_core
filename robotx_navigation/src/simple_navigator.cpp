#include <simple_navigator.h>

simple_navigator::simple_navigator()
{
    _nh.param<double>("min_search_radius", _min_search_radius, 10.0);
    _nh.param<double>("max_search_radius", _max_search_radius, 20.0);
}

simple_navigator::~simple_navigator()
{

}