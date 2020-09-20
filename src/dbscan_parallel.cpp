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

    clock_t startTime, endTime;
    startTime = clock();//计时开始
    vector<thread> threads;
    for(auto i = 0; i < 16; i++){
        threads.emplace_back(subThread,i);
    }
    for(auto i = 0; i < 16; i++){
        threads[i].join();
    }
    endTime = clock();//计时结束
    cout << "The parallel run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    startTime = clock();//计时开始
    vector<int> nums;
    for(auto i = 0; i < 16; i++){
        nums.emplace_back(i);
    }
    for(auto i = 0; i < 16; i++){
        //ROS_INFO("i am thread2 %d",i);
        cout<<i;
        clock_t st;
        st = clock();
        while((double)(clock() - st) / CLOCKS_PER_SEC < 0.01){}
    }

    endTime = clock();//计时结束
    cout << "The non-parallel run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;



    return;
}

void ParallelDbscanDriver::subThread(int i) {
    //ROS_INFO("i am thread %d",i);
    cout<<this_thread::get_id()<<endl;
    clock_t st;
    st = clock();
    while((double)(clock() - st) / CLOCKS_PER_SEC < 0.01){}
    return;
}

