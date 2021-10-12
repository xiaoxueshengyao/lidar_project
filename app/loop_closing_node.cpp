/*****
 * Description:回环检测节点文件
 * Data:1012
 * ****/
#include <ros/ros.h>
#include "glog/logging.h"

#include "general_models/tools/global_path.h"
#include "mapping/loop_closing/loop_closing_flow.hpp"


using namespace lidar_project;

int main(int argc, char** argv){

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc,argv,"loop_closing_node");
    ros::NodeHandle nh;

    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr_ = std::make_shared<LoopClosingFlow>(nh);

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();

        loop_closing_flow_ptr_->Run();
        rate.sleep();
    }

    return 0;
}