//
// Created by xcy on 2020/9/19.
//

#include "dbscan_parallel.h"


void ParallelDbscanDriver::dbscanClustering(sensor_msgs::PointCloud &PCL) {
    ROS_ERROR("GOGOGOGGOGOGOGOGOGOGOOGGOGOOGOGOGOGOGOOGOGo");
    mainThread();
    ROS_ERROR("HAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHa");
    return;
}

void ParallelDbscanDriver::mainThread() {
    ROS_INFO("num thread max: %d", thread::hardware_concurrency());

    vector<thread> threads;
    for(auto i = 0; i < 100; i++){
        threads.emplace_back(subThread,i);
    }
    for(auto i = 0; i < 100; i++){
        //threads[i].join();
    }
    return;
}

void ParallelDbscanDriver::subThread(int i) {
    ROS_INFO("i am thread %d",i);
    return;
}

