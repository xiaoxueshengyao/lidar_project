/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * ***/


#include "kitti_data/gnss_data.hpp"
#include <sensor_msgs/NavSatFix.h>   //gps话题
#include <iostream>
#include <ros/ros.h>
#include "glog/logging.h"


//类的静态成员在类外初始化
bool lidar_project::GNSSData::origin_pose_inited = false;
//坐标转化，经纬度转换为xyz
GeographicLib::LocalCartesian lidar_project::GNSSData::geo_converter;

namespace lidar_project{

void GNSSData::InitOriginPose(){
    geo_converter.Reset(latitude,longitude,altitude);
    origin_pose_inited = true;
}


void GNSSData::UpdateXYZ(){
    if(!origin_pose_inited){
        LOG(WARNING) << "Has not initialized";
    }
    geo_converter.Forward(latitude,longitude,altitude,local_x,local_y,local_z);
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData,std::deque<GNSSData>& SyncedData,double sync_time){
    //对GNSS数据进行差值，进行时间同步，找到与参考时间前后相邻的两个数据，进行差值
    //如果左右中有一个和参考时间差的比较大，说明有数据丢失，不太合适作差值
    // std::cout<<"Ready to synce GNSS"<<std::endl;
    
    while (UnsyncedData.size() >= 2)
    {   
        // std::cout<<"All data size = "<<UnsyncedData.size()<<std::endl;
        if(UnsyncedData.front().time > sync_time){
            // std::cout<<"000000"<<std::endl;
            // std::cout<<"---------"<<std::fixed<<UnsyncedData.at(0).time<<std::endl;
            return false;}
        if(UnsyncedData.at(1).time < sync_time)
            // std::cout<<"---------"<<std::fixed<<UnsyncedData.at(0).time<<std::endl;
            std::cout<<"---------"<<std::fixed<<UnsyncedData.at(1).time<<std::endl;
            UnsyncedData.pop_front();
            continue;//这里会出现死循环
        if(sync_time - UnsyncedData.front().time > 0.2){
            UnsyncedData.pop_front();
            // std::cout<<"222222"<<std::endl;
            break;
        }
        if(UnsyncedData.at(1).time - sync_time > 0.2){
            UnsyncedData.pop_front();//考虑可以留到下一个同步时间考虑
            // std::cout<<"333333"<<std::endl;
            break;

        }
        break;
    }
    if(UnsyncedData.size() < 2)  
        return false;

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
    // std::cout<<"Got gnss Synced data"<<std::endl;
    return true;   
}

}