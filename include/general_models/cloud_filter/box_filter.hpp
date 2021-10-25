/*****
 * Description:从点云中截取一个立方体部分，用于局部的定位
 * Data:1019
 * ******/

#ifndef BOX_FILTER_HPP_
#define BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>
#include "general_models/cloud_filter/filter_interface.hpp"

namespace lidar_project{
class BoxFilter : public CloudFilterInterface{
    public:
        BoxFilter(YAML::Node node);
        BoxFilter() = default;

        bool Filter(const CloudData::CloudPtr& input_cloud_ptr, CloudData::CloudPtr& filtered_cloud_ptr) override;

        void SetSize(std::vector<float> size);
        void SetOrigin(std::vector<float> origin);
        std::vector<float> GetEdge();

    private:
        void CalculateEdge();

    private:
        pcl::CropBox<CloudData::PointT> pcl_box_filter_;//放的是xyz

        std::vector<float> origin_;
        std::vector<float> size_;
        std::vector<float> edge_;
};
}
#endif