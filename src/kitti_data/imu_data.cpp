/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * 0605奇怪的东西，本来没办法进行同步数据，换了原来的版本，可以
 * 对比后法线没什么不同，再换回来，又可以了，很奇怪
 * ***/

#include "kitti_data/imu_data.hpp"
#include <cmath>
#include "glog/logging.h"

namespace lidar_project{

Eigen::Matrix3f IMUData::GetRotateMat(){
    Eigen::Quaterniond q(orientation.w,orientation.x,orientation.y,orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
    return matrix;
}


//针对IMU数据进行同步
bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time){
    //std::cout<<"Ready to synce IMU"<<std::endl;
    while (UnsyncedData.size() >= 2)
    {
        if(UnsyncedData.front().time > sync_time)
            return false;
        if(UnsyncedData.at(1).time < sync_time){
            UnsyncedData.pop_front();
            continue;
        }
        if(sync_time - UnsyncedData.front().time > 0.2){
            UnsyncedData.pop_front();
            break;
        }
        if(UnsyncedData.at(1).time - sync_time > 0.2){
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if(UnsyncedData.size() < 2 ){
        return false;
    }

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;//构造同步后的数据

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;

    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.linear_acceleration.z * back_scale;
    
    //对于四元数的差值有线性和球面差值，球面的精度更高，但是当两个四元数相差不大时，二者精度相当
    //对于相邻姿态，相差不大，就用线性差值了
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    //线性差值后要归一化才能进行后续运算
    synced_data.orientation.Normalize();
    
    SyncedData.push_back(synced_data);
    //std::cout<<"Got IMU Synced data"<<std::endl;
    return true;
    

}



}
