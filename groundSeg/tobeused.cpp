#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include "yaml-cpp/yaml.h"
#include "ground_segmentation.h"

using namespace std;


//自己重写的读参数的模板函数，形参依次是，参数的主体（YAML::Node)、参数名（string）、默认值（和参数的类型一致）
template<typename T>
T getParam(const YAML::Node& node,const string& name,const T& defaultValue)
{
    T v;
    try {
        v=node[name].as<T>();//读取参数
        //std::cout<<"Found parameter: "<<name<<",\tvalue: "<<v<<std::endl;//终端提示读取成功
    } catch (std::exception e) {//找不到该参数的话，将返回默认值
        v=defaultValue;
        //std::cout<<"Cannot find parameter: "<<name<<",\tassigning  default: "<<v<<std::endl;
    }
    return v;
}
int main(int argc, char** argv)
{
    string cloud_file = "../data/kitti.ply";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if(pcl::io::loadPLYFile(cloud_file, cloud) == -1){
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }

    GroundSegmentationParams params;
    // 加载参数文件
    YAML::Node test_config = YAML::LoadFile("../config/segmentation_params.yaml");
    params.visualize = getParam<bool>(test_config,"visualize",params.visualize);
    params.n_bins = getParam<float>(test_config,"n_bins",params.n_bins);
    params.n_segments = getParam<float>(test_config,"n_segments",params.n_segments);
    params.max_dist_to_line = getParam<float>(test_config,"max_dist_to_line",params.max_dist_to_line);
    params.max_slope = getParam<float>(test_config,"max_slope",params.max_slope);
    params.max_long_height = getParam<float>(test_config,"max_long_height",params.max_long_height);
    params.max_start_height = getParam<float>(test_config,"max_start_height",params.max_start_height);
    params.sensor_height = getParam<float>(test_config,"sensor_height",params.sensor_height);
    params.line_search_angle = getParam<float>(test_config,"line_search_angle",params.line_search_angle);
    params.max_long_height = getParam<float>(test_config,"max_long_height",params.max_long_height);
    params.n_threads = getParam<float>(test_config,"n_threads",params.n_threads);
    // 需要平方的参数
    float r_min = getParam<float>(test_config,"r_min",0.5);
    float r_max = getParam<float>(test_config,"r_max",50);
    float max_fit_error = getParam<float>(test_config,"max_fit_error",0.05);
    params.r_min_square = r_min*r_min;
    params.r_max_square = r_max*r_max;
    params.max_error_square = max_fit_error * max_fit_error;

    GroundSegmentation segmenter(params);
    std::vector<int> labels;

    segmenter.segment(cloud, &labels);
    
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (labels[i] == 1) ground_cloud.push_back(cloud[i]);//1：标记为地面点
        else obstacle_cloud.push_back(cloud[i]);
    }
    

    return 0;
}
