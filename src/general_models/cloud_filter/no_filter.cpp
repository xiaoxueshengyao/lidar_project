/***
 * Description:不适用滤波，为了提高回环的位姿匹配准确率，点云不滤波
 * Data:1011
 * ***/

#include "general_models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace lidar_project{
//构造函数，啥也不干
NoFilter::NoFilter(){
    
}

/***
 * Description:不滤波，酒把输入点云赋值给输出点云指针
 * Input::input_cloud_ptr
 * Output: filtered_cloud_ptr
 * ***/
bool NoFilter::Filter(const CloudData::CloudPtr& input_cloud_ptr,CloudData::CloudPtr& filtered_cloud_ptr){
    filtered_cloud_ptr.reset(new CloudData::CloudPointT(*input_cloud_ptr));
    return true;
}



}