#ifndef GRIDMAPMANAGE_H
#define GRIDMAPMANAGE_H

#include <pcl/ModelCoefficients.h>        // 模型系数定义头文件
#include <pcl/filters/project_inliers.h>  // 投影滤波类头文件
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <opencv2/opencv.hpp>

#include "gridMap.h"

struct GridMapParams {
    double max_x;                     
    double min_x;
    double max_y;
    double min_y;
    double max_z;
    double min_z;
    double grid_scale;                    
    int n_gridmap_x;
    int n_gridmap_y;
    int n_pixel_per_grid;
    int n_ground_estimate;
    float plane_dist_threshold;
    float max_deviation_deg;

    GridMapParams():
        max_x(11),                    //直通滤波的数值范围
        min_x(-3.0),
        max_y(6.0),
        min_y(-6.0),
        max_z(0.4),
        min_z(-1.5),                   // min_z是雷达离地面高度
        n_gridmap_x(226),
        n_gridmap_y(226),
        grid_scale(0.1),
        n_pixel_per_grid(2),
        n_ground_estimate(40),
        plane_dist_threshold(0.05),
        max_deviation_deg(10) {}

};


class GridMapManage
{
private:
    const GridMapParams params_;
    GridMap gridMap_;
public:
    GridMapManage(const GridMapParams& params_ = GridMapParams());
    // 平面投影滤波
    void groundCastFilter(const PointCloud::Ptr& inputCloud, pcl::ModelCoefficients& coefficients,
                       const PointCloud::Ptr& outputCloud);
    // 根据地面点云估计地面方程参数
    void groundEquationEstimate(PointCloud::Ptr& pointcloud, pcl::ModelCoefficients& PlaneCoeff2Show);

    //将投影的平面点云转换为栅格地图
    void planeCloud2Gridmap(const PointCloud::Ptr& map_base_plane);

    //将目标点真实雷达坐标转换到栅格地图中
    void RealCoord2GridCoord(float x, float y, int &coloum_grid, int &row_grid);
    
    //将栅格地图转换为Mat格式，用于opencv显示
    cv::Mat gridMap2Mat();
    // 获取雷达在栅格地图中的信息
    int getLaserPosNumX(){ return gridMap_.getLaserNumX(); }
    int getLaserPosNumY(){ return gridMap_.getLaserNumY(); }
    // 以向量形式返回删格地图
    void returnGridmapVector(vector<vector<int>>& mapMatrix){
        mapMatrix = gridMap_.returnGridmapVector();
    }

private:
    // 使用ransac方法获取平面方程数据
    void planeMatchRANSAC(PointCloud::Ptr& pointcloud, std::vector<int>& inliers, pcl::ModelCoefficients& PlaneCoeff2Show);
    //判断线段是否经过矩形 矩形顶点 x最小值 y最小值        长       宽   过原点线段端点 x    y ; 经过该矩形为true  不经过为false
    bool LineRecJudge (double x, double y, double deltaX, double deltaY, double EndX, double EndY);
    //判断点在线段哪一侧      横坐标   纵坐标    端点横坐标   端点纵坐标; 上侧为 true , 下侧为false;
    bool LinePointJudge (double x, double y, double EndX, double EndY);
};

#endif