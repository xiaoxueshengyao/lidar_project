/****
 * 数据处理，包括订阅、时间同步、点云畸变补偿、传感器坐标系统一
 * 0616
 * ****/
/***
 * 0824__Capta1nY__数据预处理节点
 * 完成包括数据读取，验证，同步等功能
 * 0902 更改了所有的智能指针与Eigen的语法
 * ****/

#include <ros/ros.h>
#include "glog/logging.h"

#include "data_pretreatment/data_pretreat_flow.hpp"
#include "general_models/tools/global_path.h"

using namespace lidar_project;

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]); 
    FLAGS_log_dir = WORK_SPACE_PATH  +"/Log";
    FLAGS_alsologtostderr = 1;
    
    ros::init(argc,argv,"data_pretreat_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic",cloud_topic,"/synced_cloud");

    //创建数据预处理流程对象
    // std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::shared_ptr<DataPretreatFlow>(new DataPretreatFlow(nh,cloud_topic));

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        data_pretreat_flow_ptr->Run();          //流程
        rate.sleep();

    }

    return 0;

}