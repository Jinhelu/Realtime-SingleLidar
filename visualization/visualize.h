#ifndef VISUALIZE_H_
#define VISUALIZE_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>
#include "utils/common.h"


class VisualTool{
public:
    // 构造函数完成pcl_viewer的初始化
    VisualTool();
    // visualizePointCloud可视化点云， 将点云和id添加到可视化显示器之中
    void visualizePointCloud(const PointCloud::ConstPtr& cloud, const std::string& id, int viewport = 0);
    
    // visualize可视化:在这里，将pcl_viewer中需要设定的内容都设定好
    void visualize(const PointCloud::ConstPtr& ground_cloud,
                                    const PointCloud::ConstPtr& obstacle_cloud, int viewport = 0);
    // updateVisual点云可视化更新
    void updateVisual(const PointCloud::ConstPtr& cloud, const std::string& id);
    // addSphere在视图中添加球体
    void addSphere(const PointT& center, double radius, double r, double g, double b, const std::string &id, int viewport = 0);
    // 等待鼠标键盘操作
    void spinOnce();
private:
    // 可视化的设定
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; //构建一个pcl_viewer
};

#endif