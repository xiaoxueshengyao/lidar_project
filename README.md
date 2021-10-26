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

3、glog库、地理信息库、yaml-cpp库均可自行安装
yaml如果源码安装，编译可能会出现问题，是因为ubuntu本身有yaml库，卸载掉就好

4、使用g2o作为后端，建图效果明显优化，结果如下所示

![建图1_proc](https://user-images.githubusercontent.com/33504360/136660507-8017a669-e28c-4add-99f0-b702093ecc0b.jpg)
![建图2_proc](https://user-images.githubusercontent.com/33504360/136660515-9445e025-9d37-42d4-a431-55a4fa8ee6bf.jpg)

5、建图后的定位
建图完成后，使用pcd地图作为导航地图，为加快配准速度，使用当前帧和局部地图的scan_to_map作为定位的input和target，算法仍然采用NDT，定位过程如图
![定位](https://user-images.githubusercontent.com/33504360/138859745-90152ceb-9982-4b15-8997-03ea576b229f.png)
