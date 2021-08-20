/****
* Description:A test of how to read kitti data
* Author:Capta1nY
* Data:0329
* 0604之前一直用这个，编译没问题，但是运行时，gnss的同步永远完不成，
* 没有找到具体的原因，不知道是不是ｃ++多文件和单文件编译的问题 
* ***/


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>         //imu话题
#include <sensor_msgs/NavSatFix.h>   //gps话题
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <iomanip>

#include "kitti_data.hpp"
#include "glog/logging.h"



namespace lidar_project{
//类的静态成员在类外初始化
bool lidar_project::GNSSData::origin_pose_inited = false;
//坐标转化，经纬度转换为xyz
GeographicLib::LocalCartesian lidar_project::GNSSData::geo_converter;

void GNSSData::InitOriginPose(){
    geo_converter.Reset(latitude,longitude,altitude);
    origin_pose_inited = true;
};


void GNSSData::UpdateXYZ(){
    if(!origin_pose_inited){
        LOG(WARNING) << "Has not initialized";
    }
    geo_converter.Forward(latitude,longitude,altitude,local_x,local_y,local_z);
};

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData,std::deque<GNSSData>& SyncedData,double sync_time){
    //对GNSS数据进行差值，进行时间同步，找到与参考时间前后相邻的两个数据，进行差值
    //如果左右中有一个和参考时间差的比较大，说明有数据丢失，不太合适作差值
    std::cout<<"Ready to synce GNSS"<<std::endl;
    
    while (UnsyncedData.size() >= 3)
    {   std::cout<<"All data size = "<<UnsyncedData.size()<<std::endl;
        if(UnsyncedData.front().time > sync_time){
            std::cout<<"000000"<<std::endl;
            std::cout<<"---------"<<std::fixed<<UnsyncedData.at(0).time<<std::endl;
            return false;}
        if(UnsyncedData.at(1).time < sync_time)
            std::cout<<"---------"<<std::fixed<<UnsyncedData.at(0).time<<std::endl;
            std::cout<<"---------"<<std::fixed<<UnsyncedData.at(1).time<<std::endl;
            UnsyncedData.pop_front();
            continue;//这里会出现死循环
        if(sync_time - UnsyncedData.front().time > 0.2){
            UnsyncedData.pop_front();
            std::cout<<"222222"<<std::endl;
            break;
        }
        if(UnsyncedData.at(1).time - sync_time > 0.2){
            UnsyncedData.pop_front();//考虑可以留到下一个同步时间考虑
            std::cout<<"333333"<<std::endl;
            break;

        }
        break;
    }
    if(UnsyncedData.size() < 2)  return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_x = front_data.local_x * front_scale + back_data.local_x * back_scale;
    synced_data.local_y = front_data.local_y * front_scale + back_data.local_y * back_scale;
    synced_data.local_z = front_data.local_z * front_scale + back_data.local_z * back_scale;

    SyncedData.push_back(synced_data);
    std::cout<<"Got gnss Synced data"<<std::endl;
    return true;
    


};

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




//针对线速度和角速度进行同步，做差值运算
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData,std::deque<VelocityData>& SyncedData,double sync_time){
        //找到与同步时间相邻的两个数据
        //如果两个时间差距过大，说明中间有缺失的数据，不宜做差值，做了也不准
        //核心就是让第一个数据的时间早于同步时间，第二个数据晚于同步时间
        //std::cout<<"Ready to synce Velocity"<<std::endl;
        while (UnsyncedData.size() >= 2)
        {
            if(UnsyncedData.front().time > sync_time) //如果最前面的数据时间晚于参考时间
                return false;
            if(UnsyncedData.at(1).time < sync_time){    //如果第一个早于参考时间（正常情况），但是第二个也早于
                UnsyncedData.pop_front();
                continue;
            }
            if(sync_time - UnsyncedData.front().time > 0.2){ //如果前两个的数据的时间卡在两边，但是最前面的数据离同步时间过长
                UnsyncedData.pop_front();
                break;
            }
            if(UnsyncedData.at(1).time - sync_time > 0.2){
                UnsyncedData.pop_front();
                break;
            }
            break;
        
        }
        if(UnsyncedData.size() < 2) return false;//只有俩数据时没法做差值

        VelocityData front_data = UnsyncedData.at(0);
        VelocityData back_data = UnsyncedData.at(1);
        VelocityData synced_data;

        //差值两个数据的系数
        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;

        //差值计算公式a*(1-t) + b*t,为什么呢，这样当t=0的时候就还是0，符合实际情况
        synced_data.linear_velocity.x = front_scale * front_data.linear_velocity.x + back_scale * back_data.linear_velocity.x;
        synced_data.linear_velocity.y = front_scale * front_data.linear_velocity.y + back_scale * back_data.linear_velocity.y;
        synced_data.linear_velocity.z = front_scale * front_data.linear_velocity.z + back_scale * back_data.linear_velocity.z;
        synced_data.angular_velocity.x = front_scale * front_data.angular_velocity.x + back_scale * back_data.angular_velocity.x;
        synced_data.angular_velocity.y = front_scale * front_data.angular_velocity.y + back_scale * back_data.angular_velocity.y;
        synced_data.angular_velocity.z = front_scale * front_data.angular_velocity.z + back_scale * back_data.angular_velocity.z;   

        SyncedData.push_back(synced_data);
        //std::cout<<"Got Velocity Synced data"<<std::endl;
        return true;                                                 
                            
                            
};


}

