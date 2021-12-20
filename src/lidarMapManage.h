#ifndef LIDARMAPMAG_H
#define LIDARMAPMAG_H
#include <thread>
#include "utils/common.h"
#include "utils/pointCloudQueue.h"
#include "../RSDecoder/input.h"
#include "../RSDecoder/rslidar_decoder.hpp"
#include "../RSDecoder/rslidar_packet.h"

// initRSLidar16Param 16线雷达参数初始化
void initRSLidar16Param(robosense::rslidar::ST_Param &lidarParam);

// listenRSLidar 监听获取雷达信息
void listenRSLidar(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder, robosense::rslidar_input::Input &InputObj,
              PointCloudQueue<vector<PointCloud_I>, double, long long>& PC_I_Ptr_queue, int csanIDTotal,
              bool offLineFlag, int frequence, volatile bool &SensorFlag);

// getLidarFrame 获取雷达单帧数据
bool getLidarFrame(vector<PointCloud_I>& receiver, robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,
                 robosense::rslidar_input::Input &InputObj, double &timestampFirst,long long &PCTime_u,
                 bool offLineFlag,int frequence);
#endif