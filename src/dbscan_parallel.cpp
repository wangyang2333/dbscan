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
    thread t(subThread);
    t.join();
    return;
}

void ParallelDbscanDriver::subThread() {
    while(true){
        cout<<"hello"<<endl;
    }

    return;
}

