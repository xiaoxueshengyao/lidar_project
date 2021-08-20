/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * ***/

#ifndef GNSS_DATA_HPP_
#define GNSS_DATA_HPP_

#include "GeographicLib/LocalCartesian.hpp"
#include <deque>


namespace lidar_project{

class GNSSData{
    public:
      double time = 0.0;        
      double longitude = 0.0;   //东经degrees
      double latitude = 0.0;    //北纬degrees
      double altitude = 0.0;    //海拔m
      double local_x = 0.0;     //经纬度转换为笛卡尔坐标系xyz
      double local_y = 0.0;
      double local_z = 0.0;
      int status = 0;
      int service = 0;

    private:
      static GeographicLib::LocalCartesian geo_converter;  
      static bool origin_pose_inited;   //静态成员，类初始化
    
    public:
      //初始化位姿
      void InitOriginPose();
      //转换为xyz
      void UpdateXYZ();
      //同步数据
      static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData,double sync_time);


};

}

#endif