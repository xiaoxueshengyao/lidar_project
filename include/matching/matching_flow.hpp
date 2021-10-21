/*****
 * Description: 地图匹配定位流程管理
 * Data: 1019
 * ****/

#ifndef MATCHING_FLOW_HPP_
#define MATCHING_FLOW_HPP_

#include <ros/ros.h>
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"

#include "matching/matching.hpp"


namespace lidar_project{
class MatchingFlow{
    public:
        MatchingFlow(ros::NodeHandle& nh);
        bool Run();


    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateMatching();
        bool PublishData();

    private:
        //订阅
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;

        //发布
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;

        //匹配定位
        std::shared_ptr<Matching> matching_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> gnss_data_buff_;

        CloudData current_cloud_data_;
        PoseData current_gnss_data_;

        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

};
}





#endif