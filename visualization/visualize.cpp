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
    viewer_->spinOnce(1000);
    /*另起线程，可以实现非阻塞式
    while (!viewer_->wasStopped ()){
        viewer_->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    */
}

// 显示图像
void VisualTool::imShowCVMat(const cv::String& s, cv::Mat& CVMat){
    cv::imshow(s, CVMat);
}

// 图像显示延迟
void VisualTool::myWaitKey(const int delay){
    cv::waitKey(delay);
}

// 显示起点和终点图像
void VisualTool::pointShow(int flag, int x, int y, int delta, const cv::Mat& mapImage, const cv::String& ImageName){
    if(flag != 0 && flag != 1) cerr << "输入起点或者终点标志位有误！" << endl;
    else{
        // flag=0表示是起点
        if(flag == 0){
            cv::Mat StartIcon = cv::imread(startIconAddr_, 0);
            if(StartIcon.data == nullptr) cout<< "图标文件未找到！"<< endl;
            cv::Mat imageROI = mapImage(cv::Rect(x*delta-StartIcon.cols/2, y*delta-StartIcon.rows/2,
                                        StartIcon.cols, StartIcon.rows));
            cv::addWeighted(imageROI, 0.3, StartIcon, 0.7, 0., imageROI);
            cv::imshow(ImageName, mapImage);
        }
        // flag=1表示是终点
        else{
            cv::Mat StartIcon = cv::imread(endIconAddr_, 0);//设为0 通道数为1
            if(StartIcon.data == nullptr) cout<< "图标文件未找到！"<< endl;
            cv::Mat imageROI = mapImage(cv::Rect(x*delta-StartIcon.cols/2, y*delta-StartIcon.rows/2,
                                        StartIcon.cols, StartIcon.rows));
            cv::addWeighted(imageROI, 0.3, StartIcon, 0.7, 0., imageROI);
            cv::imshow(ImageName,mapImage);
        }
    }
}