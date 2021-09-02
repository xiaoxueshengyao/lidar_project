/****
 * 数据处理，包括订阅、时间同步、点云畸变补偿、传感器坐标系统一
 * 0616
 * ****/
/***
 * 0824__Capta1nY__数据预处理节点
 * 完成包括数据读取，验证，同步等功能
 * ****/

#include <ros/ros.h>
#include "glog/logging.h"

#include "data_pretreatment/data_pretreat_flow.hpp"

using namespace lidar_project;

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]); 
    
    ros::init(argc,argv,"data_pretreat_node");
    ros::NodeHandle nh;

    //创建数据预处理流程对象
    // std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::shared_ptr<DataPretreatFlow>(new DataPretreatFlow(nh));

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        data_pretreat_flow_ptr->Run();          //流程
        rate.sleep();

    }

    return 0;

}