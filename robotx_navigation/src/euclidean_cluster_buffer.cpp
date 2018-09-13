#include <euclidean_cluster_buffer.h>

euclidean_cluster_buffer::euclidean_cluster_buffer(double buffer_length) : _tf_listener(_tf_buffer), _buffer_length(buffer_length)
{
    
}

euclidean_cluster_buffer::~euclidean_cluster_buffer()
{
    
}

void euclidean_cluster_buffer::add_cluster_data(cluster_data data)
{
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = _tf_buffer.lookupTransform("world", data.point.header.frame_id, data.point.header.stamp);
        //ROS_ERROR_STREAM(transform_stamped);
        //ROS_ERROR(transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
    _mtx.lock();
    ros::Time now = ros::Time::now();
    std::vector<cluster_data> new_buffer;
    //new_buffer.puh
    for(auto cluster_itr = _buffer.begin(); cluster_itr != _buffer.end(); cluster_itr++)
    {
        if(now - _buffer_length < cluster_itr->point.header.stamp)
        {
            new_buffer.push_back(*cluster_itr);
        }
    }
    //_buffer = new_buffer;
    _mtx.unlock();
    return;
}