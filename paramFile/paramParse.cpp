#include "paramParse.h"

//读参数的模板函数，形参依次是，参数的主体（YAML::Node)、参数名（string）、默认值（和参数的类型一致）
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
void getParameter(const string& addr, InitParam& params){
    //加载参数文件
    YAML::Node test_config = YAML::LoadFile("../paramFile/parameter.yaml");
    params.Visualize = getParam<bool>(test_config,"Visualize",params.Visualize);
    params.SorVoxLeafSize = getParam<float>(test_config,"SorVoxLeafSize",params.SorVoxLeafSize);
    params.ThresholdIntensity = getParam<float>(test_config,"ThresholdIntensity",params.ThresholdIntensity);
    params.ErrrorPointSearchRadius = getParam<float>(test_config,"ErrrorPointSearchRadius",params.ErrrorPointSearchRadius);
    params.ErrrorPointNearNum = getParam<float>(test_config,"ErrrorPointNearNum",params.ErrrorPointNearNum);
    params.StdTargetSize = getParam<float>(test_config,"StdTargetSize",params.StdTargetSize);
    params.max_x = getParam<float>(test_config,"max_x",params.max_x);
    params.min_x = getParam<float>(test_config,"min_x",params.min_x);
    params.max_y = getParam<float>(test_config,"max_y",params.max_y);
    params.min_y = getParam<float>(test_config,"min_y",params.min_y);
    params.max_z = getParam<float>(test_config,"max_z",params.max_z);
    params.min_z = getParam<float>(test_config,"min_z",params.min_z);
    params.GridmapNum_x = getParam<float>(test_config,"GridmapNum_x",params.GridmapNum_x);
    params.GridmapNum_y = getParam<float>(test_config,"GridmapNum_y",params.GridmapNum_y);
    params.GridScale = getParam<float>(test_config,"GridScale",params.GridScale);
    params.PixelPerGrid = getParam<float>(test_config,"PixelPerGrid",params.PixelPerGrid);
    params.AboveGround_NoPrecise = getParam<float>(test_config,"AboveGround_NoPrecise",params.AboveGround_NoPrecise);
    params.AboveGround = getParam<float>(test_config,"AboveGround",params.AboveGround);
    params.GroundEstimateNum = getParam<float>(test_config,"GroundEstimateNum",params.GroundEstimateNum);
    params.OutPlaneDistance = getParam<float>(test_config,"OutPlaneDistance",params.OutPlaneDistance);
    params.CutAngleYaw = getParam<float>(test_config,"CutAngleYaw",params.CutAngleYaw);
    params.CutAnglePitch = getParam<float>(test_config,"CutAnglePitch",params.CutAnglePitch);
    params.MaxDeviaAngle_deg = getParam<float>(test_config,"MaxDeviaAngle_deg",params.MaxDeviaAngle_deg);
}


