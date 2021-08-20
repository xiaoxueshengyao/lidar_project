/***
 * Des:创建文件
 * Aut:Capta1nY
 * Dat:0604
 * ***/

#include "general_models/file_manager/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_project{
//存在文件
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path){
    ofs.open(file_path.c_str(),std::ios::app);
    if(!ofs){
        LOG(WARNING)<<"No File "<<file_path;
        return false;
    }

    return true;
}

//是否存在路径，没有就创建
bool FileManager::InitDirectory(std::string directory_path, std::string use_for){
    if(boost::filesystem::is_directory(directory_path)){
        boost::filesystem::remove_all(directory_path+"/tail");
        LOG(INFO) << use_for <<" 存放地址: "<<std::endl<<directory_path<<std::endl<<std::endl;

        return true;
    }

    return CreateDirectory(directory_path,use_for);
}


//创建文件
bool FileManager::CreateDirectory(std::string directory_path){
    if(!boost::filesystem::is_directory(directory_path)){
        boost::filesystem::create_directories(directory_path);
    }
    if(!boost::filesystem::is_directory(directory_path)){
        LOG(WARNING)<<"无法创建文件  "<<directory_path;
        return false;
    }

    return true;
}


bool FileManager::CreateDirectory(std::string directory_path,std::string use_for){
    if(!boost::filesystem::is_directory(directory_path)){
        boost::filesystem::create_directories(directory_path);
    }
    if(!boost::filesystem::is_directory(directory_path)){
        LOG(WARNING)<<"无法创建文件  "<<directory_path;
        return false;
    }

    LOG(INFO) <<"存放地址："<<std::endl<<directory_path << std::endl<<std::endl;
    

    return true;
}


}
