#include "subscriber/velocity_subscriber.hpp"

namespace lidar_project{
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
    subscriber_ = nh_.subscribe(topic_name,buff_size,&VelocitySubscriber::msg_callback,this);
}

//速度数据简单获取，放到速度对象对应数据量中
void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr){
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;

    new_velocity_data_.push_back(velocity_data);
}

//每次把订阅的数据方法new_velocity_data_，然后全都放到velocity_data_buff
void VelocitySubscriber::ParaData(std::deque<VelocityData>& velocity_data_buff){
    if(new_velocity_data_.size() > 0){
        velocity_data_buff.insert(velocity_data_buff.end(),new_velocity_data_.begin(),new_velocity_data_.end());
        new_velocity_data_.clear();//只保存当前数据，存入后就清空
    }
}


}