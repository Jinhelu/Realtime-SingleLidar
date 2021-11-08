#include "visualize.h"

VisualTool::VisualTool(){
    viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
}

void VisualTool::visualizePointCloud(const PointCloud::ConstPtr& cloud, const std::string& id, int viewport) {
    viewer_->addPointCloud(cloud, id, viewport);
}

void VisualTool::updateVisual(const PointCloud::ConstPtr& cloud, const std::string& id){
    viewer_->updatePointCloud(cloud, id);
    // 注意：updatePointCloud支持更新XYZ和XYZRGB类型点云，不支持XYZI型
}

void VisualTool::visualize(const PointCloud::ConstPtr& ground_cloud,
                                const PointCloud::ConstPtr& obstacle_cloud, int viewport) {
    viewer_->setBackgroundColor (0, 0, 0, viewport);
    viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0, viewport);
    visualizePointCloud(ground_cloud, "ground_cloud");
    // 红色显示地面点云
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                1.0f, 0.0f, 0.0f,
                                                "ground_cloud", viewport);
    // 其余是障碍物
    visualizePointCloud(obstacle_cloud, "obstacle_cloud", viewport);
}

// addSphere在视图中添加球体
void VisualTool::addSphere(const PointT& center, double radius, double r, double g, double b, const std::string &id, int viewport){
    viewer_->addSphere(center, radius, r, g, b, id, viewport);
}
void VisualTool::spinOnce(){
    //viewer_->spin();// 开始显示，阻塞
    sleep(0.2);
    viewer_->spinOnce(200);
    /*另起线程，可以实现非阻塞式
    while (!viewer_->wasStopped ()){
        viewer_->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    */
}