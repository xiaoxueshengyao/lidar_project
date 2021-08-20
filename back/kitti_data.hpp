/***
 * Descriptions: all three kinds of data
 * Author: Capta1nY
 * Data: 210329
 * ****/
#ifndef KITTI_DATA_HPP_
#define KITTI_DATA_HPP_
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

#include "GeographicLib/LocalCartesian.hpp"

#include "glog/logging.h"

#include <deque>

namespace lidar_project{

//lidar data
class CloudData{
  public:
    using PointT = pcl::PointXYZ;
    using CloudPointT = pcl::PointCloud<PointT>;
    using CloudPtr = CloudPointT::Ptr;

  public:
    CloudData():cloud_ptr(new CloudPointT()){}

  public:
    double time=0.0;
    CloudPtr cloud_ptr;

};

class IMUData{
  public:
    class Orientation{
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

      public:
        //四元数归一化
        void Normalize(){
          double norm = sqrt(pow(x,2.0) + pow(y,2.0) + pow(z,2.0) + pow(w,2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    struct AngularVelocity{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct LinearAcceleration{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
  
  double time=0.0;
  Orientation orientation;
  AngularVelocity angular_velocity;
  LinearAcceleration linear_acceleration;

  public:
    Eigen::Matrix3f GetRotateMat(){
        Eigen::Quaterniond q(orientation.w,orientation.x,orientation.y,orientation.z);
        Eigen::Matrix3f matrix = q.matrix().cast<float>();
        return matrix;
    }

    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);

    
};

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

//速度（线、角）数据
class VelocityData{
  public:
    struct LinearVelocity{
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity{
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

  public:
    //同步
    static bool SyncData(std::deque<VelocityData>& UnsyncedData,
                         std::deque<VelocityData>& SyncedData,
                         double sync_time);


};



}


#endif