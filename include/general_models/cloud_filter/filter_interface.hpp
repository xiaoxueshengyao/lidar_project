/***
 * Description:为滤波模块创建接口，后屡若有其他滤波方式就继承
 * Author： Capta1nY (fork from renqian)
 * Data:0414
 * ***/

#ifndef FILTER_INTERFACE_HPP_
#define FILTER_INTERFACE_HPP_

#include "kitti_data/cloud_data.hpp"
#include <yaml-cpp/yaml.h>//yaml文件读参数，放在外面方便改写,不用总是修改源文件。


namespace lidar_project{
class CloudFilterInterface{
    public:
      virtual ~CloudFilterInterface() = default;

      virtual bool Filter(const CloudData::CloudPtr& input_cloud_ptr,CloudData::CloudPtr& filtered_cloud_ptr) = 0;//纯虚

};

}




#endif