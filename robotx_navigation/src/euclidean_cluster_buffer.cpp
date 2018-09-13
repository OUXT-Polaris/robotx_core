#include <euclidean_cluster_buffer.h>

euclidean_cluster_buffer::euclidean_cluster_buffer(double buffer_length) : _tf_listener(_tf_buffer), _buffer_length(buffer_length)
{

}

euclidean_cluster_buffer::~euclidean_cluster_buffer()
{
    
}

std::vector<boost::shared_ptr<cluster_data> > euclidean_cluster_buffer::get_cluster_data()
{
    _mtx.lock();
    ros::Time now = ros::Time::now();
    std::vector<boost::shared_ptr<cluster_data> > new_buffer;
    for(auto cluster_itr = _buffer.begin(); cluster_itr != _buffer.end(); cluster_itr++)
    {
        boost::shared_ptr<cluster_data> cluster = *cluster_itr;
        if(now - _buffer_length < cluster->point.header.stamp)
        {
            new_buffer.push_back(cluster);
        }
    }
    _mtx.unlock();
    return new_buffer;
}

void euclidean_cluster_buffer::add_cluster_data(boost::shared_ptr<cluster_data> data)
{
    _mtx.lock();
    ros::Time now = ros::Time::now();
    _buffer.push_back(data);
    std::vector<boost::shared_ptr<cluster_data> > new_buffer;
    for(auto cluster_itr = _buffer.begin(); cluster_itr != _buffer.end(); cluster_itr++)
    {
        boost::shared_ptr<cluster_data> cluster = *cluster_itr;
        if(now - _buffer_length < cluster->point.header.stamp)
        {
            new_buffer.push_back(cluster);
        }
    }
    _buffer = new_buffer;
    _mtx.unlock();
    return;
}