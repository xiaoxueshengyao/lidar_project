# noted code
参考任大佬的知乎和git，做一些简单注释和理解，向大佬学习。参考如下：

https://zhuanlan.zhihu.com/p/104791974

https://github.com/Little-Potato-1990/localization_in_auto_driving

过程中几个问题：
环境：
Ubuntu18.04
PCL 1.9
Eigen 3.3.4
1、Eigen内存对齐问题
    在编译时并未使用提供的第三方的eigen库，自己安装的3.3.4，其他的glog库、地理信息库都自行安装，
这样在编译或者运行时总会产生Eigen内存对齐的错误，做了几天，多少有些眉目。
对于类内含有Eigen对象时，需要在类内的public中添加宏声明，使用智能指针的说明都在下面的链接
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
https://blog.csdn.net/CaptainYJJ/article/details/120012332?spm=1001.2014.3001.5501
2、使用1中的类放进STL容器时也需要显示声明内存管理，代码中主要用的deque。
   std::deque<PoseData,Eigen::aligned_allocator<PoseData>> gnss_pose_data_buff_;
