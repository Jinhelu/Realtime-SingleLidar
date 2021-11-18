#ifndef LidarMapQueue_H
#define LidarMapQueue_H

#include <queue>
#include <mutex>
#include <condition_variable>
using namespace std;

//地图和规划线程间共用的类 包含起终点 二维数组
struct LidarMap{
    vector<vector<int> > map;
    int start_row;
    int start_column;
    int target_row;
    int target_column;
};


// //用于规划和雷达线程间通信的类、包括地图的二维数组、起点终点坐标
// class LidarMapQueue{
// public:
//     LidarMapQueue(int capacity){
//         _capacity = capacity;
//         _updateFlag = false;
//     }

//     void push(LidarMap &newElem){
//         lock_guard<mutex> lk(_mut);
//         _container.push(newElem);
//         _updateFlag = true;

//         if(_container.size()>_capacity){
//             _container.pop();
//         }
//     }

//     bool latestElem(LidarMap &receiver){
//         lock_guard<mutex> lk(_mut);
//         if(_container.empty()) return false;
//         receiver = _container.back();
//         _updateFlag = false;
//         return true;
//     }

//     bool isUpdated(){
//         lock_guard<mutex> lk(_mut);
//         return _updateFlag;
//     }

// private:
//     queue<LidarMap> _container;
//     bool _updateFlag;
//     mutex _mut;
//     int _capacity;
// };

//用于规划和雷达线程间通信的类、包括地图的二维数组、起点终点坐标
class LidarMap_t{
public:
    LidarMap_t(){
        _updateFlag = false;
        _container.resize(1);
    }

    void push(LidarMap &newElem){
        lock_guard<mutex> lk(_mut);
        _container[0] = newElem;
        _updateFlag = true;
        // Wake up the waiting thread.
        _cv.notify_one();
    }

    bool pop_uptodate(LidarMap &receiver){
        std::unique_lock<std::mutex> lk(_mut);
        // If the queue is empty, wait for data.
        _cv.wait(lk, [this]
        { return (_updateFlag); });
        receiver = _container[0];
        _updateFlag = false;
        return true;
    }

    bool IfUpdate(){
        lock_guard<mutex> lk(_mut);
        return _updateFlag;
    }

    void reset(){
        lock_guard<mutex> lk(_mut);
        _updateFlag = false;
    }
private:
    vector<LidarMap> _container;
    bool _updateFlag;
    mutex _mut;
    condition_variable _cv;
};

#endif
