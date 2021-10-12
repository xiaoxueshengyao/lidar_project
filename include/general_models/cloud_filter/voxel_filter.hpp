/***
 * Des: 体素滤波，采用定义的接口进行继承
 * Author： Capta1nY
 * Data: 0414
 * ***/

#ifndef VOXEL_FILTER_HPP_
#define VOXEL_FILTER_HPP_

#include "general_models/cloud_filter/filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>

namespace lidar_project{
class VoxelFilter: public CloudFilterInterface{
    public:
      VoxelFilter(const YAML::Node& node);
      VoxelFilter(float leaf_size_x,float leaf_size_y,float leaf_size_z);

      bool Filter(CloudData::CloudPtr& input_cloud_ptr,CloudData::CloudPtr& filtered_cloud_ptr) override;

    private:
      bool SetFilterParam(float leaf_size_x,float leaf_size_y,float leaf_size_z);
    
    private:
      pcl::VoxelGrid<CloudData::PointT> voxel_filter_;
};


}



#endif