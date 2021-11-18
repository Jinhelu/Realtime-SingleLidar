#ifndef VISUALIZE_H_
#define VISUALIZE_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
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
    /******* opencv显示 ********/
    // 显示起点和终点图像
    void pointShow(int flag, int x, int y, int delta, const cv::Mat& MapImage, const cv::String& ImageName);
    // OpenCV图像显示
    void imShowCVMat(const cv::String& s, cv::Mat& CVMat);
    // 图像显示延迟
    void myWaitKey(const int delay);
private:
    // 可视化的设定
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_; //构建一个pcl_viewer
    const string startIconAddr_ = "../paramFile/icon/start_15.jpg";
    const string endIconAddr_ = "../paramFile/icon/end_15.jpg";  
};

#endif