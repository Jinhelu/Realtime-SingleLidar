#include "lidarMapManage.h"


void initRSLidar16Param(robosense::rslidar::ST_Param &lidarParam){
    lidarParam.lidar = robosense::rslidar::RS_Type_Lidar32;
    lidarParam.resolution = robosense::rslidar::RS_Resolution_5mm;
    //lidarParam.intensity = robosense::rslidar::RS_INTENSITY_EXTERN;
    lidarParam.echo = robosense::rslidar::RS_Echo_Last;//RS_Echo_Strongest;
    //lidarParam.echo = robosense::rslidar::RS_Echo_Dual;
    //lidarParam.echo = robosense::rslidar::RS_Echo_Last;
    lidarParam.cut_angle = 0.1f;
    lidarParam.max_distance = 30.0f;
    lidarParam.min_distance = 0.2f;
    lidarParam.start_angle = 0.0f;//0.0f;
    lidarParam.end_angle = 360.0f;//360.0f;
    //param.cali_path = "./RS16";
}

bool getLidarFrame(vector<PointCloud_I>& receiver, robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder,
                 robosense::rslidar_input::Input &InputObj, double &timestampFirst,long long &PCTime_u,
                 bool offLineFlag,int frequence){
    double timestamp;
    bool timeFlag=true;
    for(auto &PC_I:receiver)
        PC_I.clear();

    int pktNum = 0;

    vector<PointXYZITS> pointcloudI_buf;

    while(true){
        uint8_t pkt_buf[1248];//1248
        robosense::rslidar_input::InputState bufret = InputObj.getPacket(pkt_buf);
        //pointcloudI_buf.clear();

        ///////////////////////扫描数据输出处理
        if(bufret == robosense::rslidar_input::INPUT_MSOP){
            //process MSOP packet
            //int ret = decoder.processMsopPkt(pkt_buf+12, pointcloudI_buf, timestamp);
            int ret = decoder.processMsopPkt(pkt_buf, pointcloudI_buf, timestamp);
            if (ret == robosense::rslidar::RS_Decode_ok) {
                if(timeFlag) {
                    timestampFirst=timestamp;
                    timeFlag=false;
                }
                //cout << "Pkt共计点数：" << pointcloudI_buf.size() << endl << "时间：" << timestamp << endl;
                //cout <<"**************************************************"<< endl << endl;
                //for(const auto & i : pointcloudI_buf){
                //    pointcloud_I->points.push_back(i);
                //}
                //*receiver += *pointcloud_I;//点云内容可以直接加，指针不可以直接加
                pktNum++;

            }
            else if(ret== robosense::rslidar::RS_Frame_Split){//一帧结束，新一帧开始
                struct timeval tv;
                gettimeofday(&tv,NULL);
                PCTime_u = tv.tv_sec*1000000 + tv.tv_usec;

                //cout << "一帧点数: " << pointcloudI_buf.size() << endl;
                for(int i=0;i<pointcloudI_buf.size();i++){
                    PointT_I temp;
                    double deltaTime = pointcloudI_buf[i].timeStamp - timestampFirst;//us为单位

                    //todo 去畸变写在此处
                    temp.x = pointcloudI_buf[i].x;
                    temp.y = pointcloudI_buf[i].y;
                    temp.z = pointcloudI_buf[i].z;
                    temp.intensity = pointcloudI_buf[i].intensity;
                    int scanID = pointcloudI_buf[i].scanID;

                    //对于16线，ID号不连续
                    receiver[scanID].points.push_back(temp);
                }

                if(offLineFlag) usleep(900*1000/frequence);
                //cout <<"pktNum: " << pktNum << endl;
                return true;
            }
            else{
                if(ret== robosense::rslidar::RS_Decode_Fail){
                    cout << "ERROR: packet MSOP decode error accure!" << endl;
                }
                else if(ret== robosense::rslidar::RS_Param_Invalid){
                    cout << "ERROR: input MSOP packet buffer pointer invalid!" << endl;
                }
            }
        }//INPUT_MSOP
        /*设备信息输出处理*/
        else if(bufret == robosense::rslidar_input::INPUT_DIFOP){
            int ret = decoder.processDifopPkt(pkt_buf);
            if (ret== robosense::rslidar::RS_Decode_ok){
                //cout << "RslidarInput::INPUT_DIFOP : RS_Decode_ok " << endl;
            }
            else if(ret== robosense::rslidar::RS_Frame_Split){
                cout << "Warning: packet DIFOP decode ok and match frame split!" << endl;
            }
            else{
                if(ret== robosense::rslidar::RS_Decode_Fail){
                    cout << "ERROR: packet DIFOP decode error accure!" << endl;
                }
                else if(ret== robosense::rslidar::RS_Param_Invalid){
                    cout << "ERROR: input packet DIFOP buffer pointer invalid!" << endl;
                }
            }
        }//INPUT_DIFOP
        else cout<< "Failed to get pkt_buf or resolve pkt_buf!" << endl;
    }

}

void listenRSLidar(robosense::rslidar::RSLidarDecoder<PointXYZITS> &decoder, robosense::rslidar_input::Input &InputObj,
              PointCloudQueue<vector<PointCloud_I>,double,long long>& PC_I_Ptr_queue, int csanIDTotal,
              bool offLineFlag, int frequence, volatile bool &SensorFlag){
    vector<PointCloud_I> PC_I_raw_RS(csanIDTotal);
    double timeStamp;
    long long PCTime_u;
    while(SensorFlag && getLidarFrame(PC_I_raw_RS, decoder, InputObj, timeStamp, PCTime_u, offLineFlag, frequence)){
        PC_I_Ptr_queue.push(PC_I_raw_RS,timeStamp,PCTime_u);
    }
}