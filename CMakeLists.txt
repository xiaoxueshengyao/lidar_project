cmake_minimum_required(VERSION 2.8)
project(lidar_project)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  roscpp  
  rospy  
  std_msgs 
  pcl_ros 
  geometry_msgs 
  tf 
  eigen_conversions
  message_generation
  std_srvs
)

#创建server
add_service_files(
  FILES
  saveMap.srv
  optimizeMap.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

set(ALL_TARGET_LIBRARIES "")#建立一个变量，把需要的库都放一起

#g2o库
include(cmake/g2o.cmake)

#PCL库
find_package(PCL REQUIRED)
message("PCL version  is ： ${PCL_VERSION}")
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(APPEND ALL_TARGET_LIBRARIES ${PCL_LIBRARIES})  #添加pcl库

#谷歌日志库
find_package(gflags REQUIRED)
find_package(Glog REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
if(GLOG_FOUND)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  message("Glog got =======")
endif()

list(APPEND ALL_TARGET_LIBRARIES ${GLOG_LIBRARIES} ${GFLAGS_INCLUDE_DIRS})#添加日志库


#地理信息坐标
find_package(GeographicLib REQUIRED)#需要安装对应的库
include_directories(${Geographic_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${GeographicLib_LIBRARIES} )
#原版使用
#add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
#include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)
#list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)

#参数文件读取
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES yaml-cpp)

include_directories( "/usr/include/eigen3" )
#include_directories("third_party/eigen3")
include_directories(include ${catkin_INCLUDE_DIRS})

#工作空间设定
configure_file(
  ${PROJECT_SOURCE_DIR}/include/tools/global_path.h
  ${PROJECT_SOURCE_DIR}/include/tools/global_path.h
)
#include_directories(${PROJECT_BINARY_DIR}/include)


catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
DEPENDS PCL
INCLUDE_DIRS include)


#add_executable(test_data app/test_frame.cpp src/kitti_data/gnss_data.cpp src/kitti_data/imu_data.cpp src/tf_listener.cpp)
#target_link_libraries(test_data ${catkin_LIBRARIES} 
#${PCL_LIBRARIES} 
#${Eigen_LIBRARIES}
#${GeographicLib_LIBRARIES}
#glog::glog
#)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE APP_SRCS "app/*.cpp")
#file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${APP_SRCS})
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
#对列表的操作分为读取、查找、修改、排序

#add_executable(front_end app/front_test.cpp src/kitti_data.cpp src/tf_listener.cpp src/front_end_back.cpp)
#target_link_libraries(front_end ${catkin_LIBRARIES} 
#${PCL_LIBRARIES} ${Eigen_LIBRA/home/jingwan/lslidar_ws/src
add_executable(test_frame_node app/test_frame.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog)

#数据预处理节点
add_executable(data_pretreat_node app/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
#在使用ROS的message、service、action时注意添加 add_dependencies
#添加对其它package消息的依赖，前提是已经通过find_package()引入了这个package
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog)

#前端节点
add_executable(front_end_node app/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog)

#后端节点
add_executable(back_end_node app/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog)

#可视化节点
add_executable(viewer_node app/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} glog)

