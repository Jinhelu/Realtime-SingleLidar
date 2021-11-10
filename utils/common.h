#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <cstring>
#include <vector>
#include <cmath>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


// 点云类型和点云指针类型重定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef pcl::PointXYZI PointT_I;
typedef pcl::PointCloud<PointT_I> PointCloud_I;

typedef pcl::PointXYZRGB PointT_C;
typedef pcl::PointCloud<PointT_C> PointCloud_C;

// PointXYZITS 自定义带有强度、时间戳、线号的三维点信息; 仅用于在驱动程序中读取更加详尽的信息
struct PointXYZITS{
    float x;
    float y;
    float z;
    float intensity;
    float timeStamp;
    int scanID;
};

// InitParam 算法中的参数
struct InitParam{
    float AboveGround_NoPrecise; //地面精度低时删除点距地面的高度
    float AboveGround; //地面精度正常时删除点距地面的高度
    int CutAngleYaw; //估计地面时采样的左右边缘角度（deg）
    int CutAnglePitch; //估计地面时采样的俯角（deg）
    bool visualize; // 是否进行可视化
    string pcapAddr;
    InitParam() :
        AboveGround_NoPrecise(0.8),
        AboveGround(0.25),
        CutAngleYaw(32),
        CutAnglePitch(68),
        pcapAddr("../trees.pcap"),
        visualize(false) {}
};

#endif