#ifndef POINTCLOUDQUEUE_H
#define POINTCLOUDQUEUE_H
#include <queue>
#include <mutex>
#include "common.h"

template <typename T,typename timeType,typename PCtimeType>
class PointCloudQueue{
public:
    PointCloudQueue(int capacity){
        _capacity = capacity;
    }

    void push(T &newElem,timeType &newTime,PCtimeType &PCtime){
        lock_guard<mutex> lk(_mut);
        _container.push(newElem);
        _timeContainer.push(newTime);
        _PCtimeContainer.push(PCtime);

        if(_container.size()>_capacity){
            _container.pop();
            _timeContainer.pop();
            _PCtimeContainer.pop();
        }
    }

    bool latestElem(T* receiver,timeType* timeReceiver,PCtimeType* PCtimePtr){
        lock_guard<mutex> lk(_mut);
        if(_container.empty()) return false;
        *receiver = _container.back();
        *timeReceiver = _timeContainer.back();
        *PCtimePtr = _PCtimeContainer.back();
        return true;
    }

private:
    queue<T> _container;
    queue<timeType> _timeContainer;
    queue<PCtimeType> _PCtimeContainer;
    mutex _mut;
    int _capacity;
};

#endif