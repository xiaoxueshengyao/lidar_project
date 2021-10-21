#include "subscriber/cloud_subscriber.hpp"

namespace lidar_project{

//Cloud
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh,std::string topic_name,size_t buff_size)
    :nh_(nh){
        subscriber_ = nh_.subscribe(topic_name,buff_size,&CloudSubscriber::msg_callback,this);
    }

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr){
    CloudData cloud_buff;
    cloud_buff.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr,*(cloud_buff.cloud_ptr));
    // std::cout<<"Got cloud data!!!!"<<std::endl;

    new_cloud_data_.push_back(cloud_buff);
}

/***数据存储，把订阅的点云数据放到deque中***/
void CloudSubscriber::ParaData(std::deque<CloudData>& deque_cloud_data){
    if(new_cloud_data_.size()>0){
        deque_cloud_data.insert(deque_cloud_data.end(),new_cloud_data_.begin(),new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}

    
}