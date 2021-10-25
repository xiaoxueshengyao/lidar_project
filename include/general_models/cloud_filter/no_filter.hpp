/***
 * Description:不适用滤波，为了提高回环的位姿匹配准确率，点云不滤波
 * Data:1011
 * ***/

#ifndef NO_FILTER_HPP_
#define NO_FILTER_HPP_

#include "general_models/cloud_filter/filter_interface.hpp"

namespace lidar_project{
class NoFilter : public CloudFilterInterface{
    public:
        NoFilter();

        bool Filter(const CloudData::CloudPtr& input_cloud_ptr,CloudData::CloudPtr& filtered_cloud_ptr) override;//重载滤波函数
};

}
#endif