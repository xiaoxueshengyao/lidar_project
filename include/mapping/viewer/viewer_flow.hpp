/***
 * Description:可视化显示流程管理
 * Data:0615
 * ****/


#ifndef VIEWER_FLOW_HPP_
#define VIEWER_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
//数据订阅
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"
#include "subscriber/key_frame_subscriber.hpp"
#include "subscriber/key_frames_subscriber.hpp"
//数据发布
#include "publisher/odometry_publisher.hpp"
#include "publisher/cloud_publisher.hpp"
//显示
#include "mapping/viewer/viewer.hpp"

namespace lidar_project
{
class ViewerFlow{
    public:
        ViewerFlow(ros::NodeHandle& nh);

        bool Run();
        bool SaveMap();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        // bool UpdateViewer();
        // bool PublishData();
        bool PublishGlobalData();//发布全局地图
        bool PublishLocalData();//发布局部地图


    private:
        //数据订阅
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
        
        //数据发布
        std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;

        //显示
        std::shared_ptr<Viewer> viewer_ptr_;

        //数据存放
        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData,Eigen::aligned_allocator<PoseData>> transformed_odom_buff_;
        // std::deque<PoseData> transformed_odom_buff_;
        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> optimized_key_frames_;
        std::deque<KeyFrame> all_key_frames_;

        CloudData current_cloud_data_;
        PoseData current_transformed_odom_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};    






} // namespace lidar_project

#endif