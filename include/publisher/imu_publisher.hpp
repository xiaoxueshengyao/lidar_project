/***
 * Des:在ros中发布IMU数据
 * Data:0610
 * ***/


#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "kitti_data/imu_data.hpp"

namespace lidar_project{
class IMUPublisher{
    public:
        IMUPublisher(ros::NodeHandle& nh,
                     std::string topic_name,
                     size_t buff_size,
                     std::string frame_id);
        IMUPublisher() = default;

        void Publish(IMUData imu_data);
        bool HasSubscribers();


    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;


};

}



#endif