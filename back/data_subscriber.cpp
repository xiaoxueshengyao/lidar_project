/***
 * 三种数据的订阅与处理，主要就是回调函数和存储
 * 0413
 * ***/

#include "data_subscriber.hpp"


namespace lidar_project{
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh,std::string topic_name, size_t buff_size)
    :nh_(nh){
        subscriber_ = nh_.subscribe(topic_name,buff_size,&IMUSubscriber::msg_callback,this);
    }

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr){
    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();
    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;//对角速度积分得到角度
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;//对加速度二次积分得到位移
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_data.orientation.w = imu_msg_ptr->orientation.w;
    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;

    new_imu_data_.push_back(imu_data);
}

void IMUSubscriber::ParasData(std::deque<IMUData>& deque_imu_data){//这里要在main中建个deque存储数据
    if(new_imu_data_.size()>0){
        deque_imu_data.insert(deque_imu_data.end(),new_imu_data_.begin(),new_imu_data_.end());
        new_imu_data_.clear();
    }
}


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

void CloudSubscriber::ParaData(std::deque<CloudData>& deque_cloud_data){
    if(new_cloud_data_.size()>0){
        deque_cloud_data.insert(deque_cloud_data.end(),new_cloud_data_.begin(),new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}



//速度订阅类实现
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
    subscriber_ = nh_.subscribe(topic_name,buff_size,&VelocitySubscriber::msg_callback,this);
}

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