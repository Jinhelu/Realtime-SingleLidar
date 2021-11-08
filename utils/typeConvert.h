#ifndef TYPECONVERT_H
#define TYPECONVERT_H

#include "common.h"

//带反射率的点云转换为坐标点云
void XYZI2XYZ(const PointCloud_I::Ptr& raw,PointCloud::Ptr& result){
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
void XYZI2XYZRGB(const PointCloud_I::Ptr& raw, PointCloud_C::Ptr& result){
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

#endif