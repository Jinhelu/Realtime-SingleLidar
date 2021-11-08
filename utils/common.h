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
    float SorVoxLeafSize;
    int ThresholdIntensity;
    float ErrrorPointSearchRadius;
    int ErrrorPointNearNum;
    float StdTargetSize;//对角线长度
    double max_x,min_x,max_y,min_y,max_z,min_z;//min_z是雷达离地面高度

    int GridmapNum_x, GridmapNum_y;//x,y两边的删格数目
    double GridScale;//每个删格代表的实际长度 单位：米
    int PixelPerGrid;//显示地图时候一个删格边长占据的像素

    float AboveGround_NoPrecise; //地面精度低时删除点距地面的高度
    float AboveGround; //地面精度正常时删除点距地面的高度
    int GroundEstimateNum; //估计地面时迭代随机采样的次数
    float OutPlaneDistance; //随机采样时局外点的距离
    int CutAngleYaw; //估计地面时采样的左右边缘角度（deg）
    int CutAnglePitch; //估计地面时采样的俯角（deg）
    float MaxDeviaAngle_deg; //有精度地面的最大偏离均值角度
    bool visualize; // 是否进行可视化
    InitParam() :
        SorVoxLeafSize(0.035),
        ThresholdIntensity(196),
        ErrrorPointSearchRadius(0.1),
        ErrrorPointNearNum(4),
        StdTargetSize(0.35355),
        max_x(11),
        min_x(-3.0),
        max_y(6.0),
        min_y(-6.0),
        max_z(0.4),
        min_z(-1.5),
        GridmapNum_x(226),
        GridmapNum_y(226),
        GridScale(0.1),
        PixelPerGrid(2),
        AboveGround_NoPrecise(0.8),
        AboveGround(0.25),
        GroundEstimateNum(40),
        OutPlaneDistance(0.05),
        CutAngleYaw(32),
        CutAnglePitch(68),
        MaxDeviaAngle_deg(0.3),
        visualize(false) {}
};

#endif