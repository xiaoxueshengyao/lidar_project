/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * ***/

#ifndef CLOUD_DATA_HPP_
#define CLOUD_DATA_HPP_


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


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



}


#endif