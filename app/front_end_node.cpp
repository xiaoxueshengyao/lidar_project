/***
 * Description: 更新后前端，主函数就一个对象创建了，简洁
 * Data: 0419
 * ***/

#include <ros/ros.h>
#include "glog/logging.h"
#include "mapping/front_end/front_end_flow.hpp"

//#include <lidar_project/saveMap.h>              //ros的服务头文件，不用写出实体，在cmakelists.txt中写出可在build自动生成

using namespace lidar_project;


int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/jingwan/lslidar_ws/src/lidar_project/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc,argv,"front_end_node");
    ros::NodeHandle nh;

    //ros::ServiceServer service = nh.advertiseService("save_map",save_map_callback);
    //_front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    //std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::allocate_shared<FrontEndFlow>(Eigen::aligned_allocator<FrontEndFlow>(nh));//前端流程对象
    std::shared_ptr<FrontEndFlow>  front_end_flow_ptr = std::shared_ptr<FrontEndFlow>(new FrontEndFlow(nh));//前端流程对象

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        front_end_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
    
}