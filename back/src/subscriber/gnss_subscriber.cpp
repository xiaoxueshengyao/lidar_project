#include "subscriber/gnss_subscriber.hpp"

namespace lidar_project{

//GNSS
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh,std::string topic_name,size_t buff_size)
    :nh_(nh){
        subscriber_ = nh_.subscribe(topic_name,buff_size,&GNSSSubscriber::msg_callback,this);
    }

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptr){
    GNSSData gnss_data;
    gnss_data.time = gnss_msg_ptr->header.stamp.toSec();
    gnss_data.altitude = gnss_msg_ptr->altitude;
    gnss_data.longitude = gnss_msg_ptr->longitude;
    gnss_data.latitude = gnss_msg_ptr->latitude;

    gnss_data.service = gnss_msg_ptr->status.service;     //which global navigation satellite system
    gnss_data.status = gnss_msg_ptr->status.status;       //fix type

    // std::cout<<"Got GNSS data"<<std::endl;

    new_gnss_data_.push_back(gnss_data);

}

void GNSSSubscriber::ParasData(std::deque<GNSSData>& deque_gnss_data){//像这些相同功能不同参数的可以用多态来实现，包括上面的msg_callback
    if(new_gnss_data_.size() > 0){
        deque_gnss_data.insert(deque_gnss_data.end(),new_gnss_data_.begin(),new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}



}