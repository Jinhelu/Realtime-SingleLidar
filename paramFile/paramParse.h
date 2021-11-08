#ifndef PARAMPARSE_H
#define PARAMPARSE_H

#include <iostream>
#include "yaml-cpp/yaml.h"
#include "utils/common.h"
#include "groundSeg/ground_segmentation.h"
#include "objDetect/objDetect.h"
using namespace std;

// getParam自定义读取yaml文件函数
template<typename T>
T getParam(const YAML::Node& node,const string& name,const T& defaultValue);

// getParameter使用yaml配置文件加载算法中的参数（便于tune）
void getParameter(InitParam& params);

// getGroundSegParameter 加载地面分割算法参数
void getGroundSegParameter(GroundSegmentationParams& params);

// getObjDetectParameter 加载目标检测算法参数
void getObjDetectParameter(ObjDetectParams& params);

#endif