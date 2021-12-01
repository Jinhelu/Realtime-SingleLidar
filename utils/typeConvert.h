#ifndef TYPECONVERT_H
#define TYPECONVERT_H

#include "common.h"

//带反射率的点云转换为坐标点云
inline void XYZI2XYZ(const PointCloud_I::Ptr& raw,PointCloud::Ptr& result){
    result->clear();
    for(int i=0; i < raw->size() ;i++){
        PointT p;
        p.x = raw->points[i].x;
        p.y = raw->points[i].y;
        p.z = raw->points[i].z;
        result->points.push_back(p);
    }
}

//带反射率的点云转换为颜色点云，用于显示
inline void XYZI2XYZRGB(const PointCloud_I::Ptr& raw, PointCloud_C::Ptr& result){
    result->clear();
    for(int i=0; i < raw->size();i++){
        PointT_C p;
        p.x = raw->points[i].x;
        p.y = raw->points[i].y;
        p.z = raw->points[i].z;

        if((int)raw->points[i].intensity < 90){
            p.r = int(0);
            p.g = int(0 + raw->points[i].intensity*2.82f);
            p.b = int(255 - raw->points[i].intensity*2.82f);
        }
        else if ((int)raw->points[i].intensity >= 90) {
            p.r = int((raw->points[i].intensity-90)*1.53f);
            p.g = int(255 - (raw->points[i].intensity-90)*1.53f);
            p.b = int(0);
        }
        result->points.push_back(p);
    }
}

// 点云空间坐标信息转换
inline void convertPointCloud(const PointCloud_I::Ptr& inputCloud, Eigen::Matrix3f rotationMatrix){
    for(size_t i=0; i<inputCloud->points.size(); i++){
        float &point16_x = inputCloud->points[i].x;
        float &point16_y = inputCloud->points[i].y;
        float &point16_z = inputCloud->points[i].z;

        Eigen::Vector3f point16_E(point16_x, point16_y, point16_z);
        point16_E = rotationMatrix * point16_E;
        point16_x = point16_E.x();
        point16_y = point16_E.y();
        point16_z = point16_E.z();
    }
}

#endif