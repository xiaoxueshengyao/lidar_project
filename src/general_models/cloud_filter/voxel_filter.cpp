/***
 * Des:The realization of class voxel_filter
 * Author: Capta1nY
 * Data:0414
 * ***/

#include "general_models/cloud_filter/voxel_filter.hpp"

namespace lidar_project{
VoxelFilter::VoxelFilter(const YAML::Node& node){
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    
    SetFilterParam(leaf_size_x,leaf_size_y,leaf_size_z);

}

VoxelFilter::VoxelFilter(float leaf_size_x,float leaf_size_y,float leaf_size_z){
    SetFilterParam(leaf_size_x,leaf_size_y,leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x,float leaf_size_y,float leaf_size_z){
    voxel_filter_.setLeafSize(leaf_size_x,leaf_size_y,leaf_size_z);
    std::cout<<"Voxel Filter Parameters: "<<"\n"<<leaf_size_x<<"\t"<<leaf_size_y<<"\t"<<leaf_size_z<<std::endl;
    return true;
}

/****
 * INPUT: input_cloud_ptr  输入点云
 * OUTPUT: fltered_cloud_ptr 滤波后点云
 * ****/
bool VoxelFilter::Filter(const CloudData::CloudPtr& input_cloud_ptr,CloudData::CloudPtr& filtered_cloud_ptr){
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);
    return true;
}



}