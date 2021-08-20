/***
 * 0604参考kitti_data把数据发布也分开
 * ***/

#ifndef CLOUD_PUBLISHER_HPP_
#define CLOUD_PUBLISHER_HPP_

#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include "kitti_data/cloud_data.hpp"


namespace lidar_project{

//点云发布
class CloudPublisher{
    public:
      CloudPublisher(ros::NodeHandle& nh, std::string topic_name,size_t buff_size,std::string frame_id);
      CloudPublisher() = default;
      void Publish(CloudData::CloudPtr cloud_ptr);


    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      std::string frame_id_;


};


}


#endif