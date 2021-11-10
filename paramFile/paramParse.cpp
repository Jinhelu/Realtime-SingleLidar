#include "paramParse.h"

// getParam读参数的模板函数，形参依次是，参数的主体（YAML::Node)、参数名（string）、默认值（和参数的类型一致）
template<typename T>
T getParam(const YAML::Node& node,const string& name,const T& defaultValue){
    T v;
    try {
        v=node[name].as<T>();//读取参数
        //std::cout<<"Found parameter: "<<name<<",\tvalue: "<<v<<std::endl;
    } catch (std::exception e) {//找不到该参数的话，将返回默认值
        v=defaultValue;
        //std::cout<<"Cannot find parameter: "<<name<<",\tassigning  default: "<<v<<std::endl;
    }
    return v;
}

// getParameter使用yaml配置文件加载算法中的参数（便于tune）
void getParameter(InitParam& params){
    //加载参数文件
    YAML::Node yamlNodeObj = YAML::LoadFile("../paramFile/parameter.yaml");
    params.visualize = getParam<bool>(yamlNodeObj,"visualize",params.visualize);
    params.AboveGround_NoPrecise = getParam<float>(yamlNodeObj,"AboveGround_NoPrecise",params.AboveGround_NoPrecise);
    params.AboveGround = getParam<float>(yamlNodeObj,"AboveGround",params.AboveGround);
    params.CutAngleYaw = getParam<float>(yamlNodeObj,"CutAngleYaw",params.CutAngleYaw);
    params.CutAnglePitch = getParam<float>(yamlNodeObj,"CutAnglePitch",params.CutAnglePitch);
}

// getGroundSegParameter 加载地面分割算法参数
void getGroundSegParameter(GroundSegmentationParams& params){
    //加载参数文件
    YAML::Node yamlNodeObj = YAML::LoadFile("../paramFile/groundSegParams.yaml");
    params.visualize = getParam<bool>(yamlNodeObj,"visualize",params.visualize);
    params.n_bins = getParam<float>(yamlNodeObj,"n_bins",params.n_bins);
    params.n_segments = getParam<float>(yamlNodeObj,"n_segments",params.n_segments);
    params.max_dist_to_line = getParam<float>(yamlNodeObj,"max_dist_to_line",params.max_dist_to_line);
    params.max_slope = getParam<float>(yamlNodeObj,"max_slope",params.max_slope);
    params.max_long_height = getParam<float>(yamlNodeObj,"max_long_height",params.max_long_height);
    params.max_start_height = getParam<float>(yamlNodeObj,"max_start_height",params.max_start_height);
    params.sensor_height = getParam<float>(yamlNodeObj,"sensor_height",params.sensor_height);
    params.line_search_angle = getParam<float>(yamlNodeObj,"line_search_angle",params.line_search_angle);
    params.max_long_height = getParam<float>(yamlNodeObj,"max_long_height",params.max_long_height);
    params.n_threads = getParam<float>(yamlNodeObj,"n_threads",params.n_threads);
    // 需要平方的参数
    float r_min = getParam<float>(yamlNodeObj,"r_min",0.5);
    float r_max = getParam<float>(yamlNodeObj,"r_max",50);
    float max_fit_error = getParam<float>(yamlNodeObj,"max_fit_error",0.05);
    params.r_min_square = r_min*r_min;
    params.r_max_square = r_max*r_max;
    params.max_error_square = max_fit_error * max_fit_error;
}

// getObjDetectParameter 加载目标检测算法参数
void getObjDetectParameter(ObjDetectParams& params){
    //加载参数文件
    YAML::Node yamlNodeObj = YAML::LoadFile("../paramFile/objDetectParams.yaml");
    params.intensity_threshold = getParam<float>(yamlNodeObj,"intensity_threshold",params.intensity_threshold);
    params.n_reflectPoint_closest = getParam<int>(yamlNodeObj,"n_reflectPoint_closest",params.n_reflectPoint_closest);
    params.r_reflectPoint_search = getParam<float>(yamlNodeObj,"r_reflectPoint_search",params.r_reflectPoint_search);
    params.std_target_size = getParam<float>(yamlNodeObj,"std_target_size",params.std_target_size);
    params.voxel_leaf_size = getParam<float>(yamlNodeObj,"voxel_leaf_size",params.voxel_leaf_size);
}

// getGridMapParameter 加载生成栅格地图算法参数
void getGridMapParameter(GridMapParams& params){
    YAML::Node yamlNodeObj = YAML::LoadFile("../paramFile/gridMapParams.yaml");
    params.max_x = getParam<float>(yamlNodeObj,"max_x",params.max_x);
    params.min_x = getParam<float>(yamlNodeObj,"min_x",params.min_x);
    params.max_y = getParam<float>(yamlNodeObj,"max_y",params.max_y);
    params.min_y = getParam<float>(yamlNodeObj,"min_y",params.min_y);
    params.max_z = getParam<float>(yamlNodeObj,"max_z",params.max_z);
    params.min_z = getParam<float>(yamlNodeObj,"min_z",params.min_z);
    params.grid_scale = getParam<float>(yamlNodeObj,"grid_scale",params.grid_scale);
    params.max_deviation_deg = getParam<float>(yamlNodeObj,"max_deviation_deg",params.max_deviation_deg);
    params.n_gridmap_x = getParam<float>(yamlNodeObj,"n_gridmap_x",params.n_gridmap_x);
    params.n_gridmap_y = getParam<float>(yamlNodeObj,"n_gridmap_y",params.n_gridmap_y);
    params.n_ground_estimate = getParam<float>(yamlNodeObj,"n_ground_estimate",params.n_ground_estimate);
    params.n_pixel_per_grid = getParam<float>(yamlNodeObj,"n_pixel_per_grid",params.n_pixel_per_grid);
    params.plane_dist_threshold = getParam<float>(yamlNodeObj,"plane_dist_threshold",params.plane_dist_threshold);
}