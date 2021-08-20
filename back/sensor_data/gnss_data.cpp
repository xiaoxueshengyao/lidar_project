/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "kitti_data/gnss_data.hpp"

#include "glog/logging.h"
#include <iostream>

//静态成员变量必须在类外初始化
bool lidar_project::GNSSData::origin_pose_inited = false;
GeographicLib::LocalCartesian lidar_project::GNSSData::geo_converter;

namespace lidar_project {
void GNSSData::InitOriginPose() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_pose_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_pose_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_x, local_y, local_z);
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time){
            std::cout<<"000000"<<std::endl;
            std::cout<<"---------"<<std::fixed<<UnsyncedData.at(0).time<<std::endl;
            return false;}
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            std::cout<<"---------"<<std::fixed<<UnsyncedData.at(0).time<<std::endl;
            //std::cout<<"---------"<<std::fixed<<UnsyncedData.at(1).time<<std::endl;
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            std::cout<<"222222"<<std::endl;
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            std::cout<<"333333"<<std::endl;
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
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
    
    return true;
}
}