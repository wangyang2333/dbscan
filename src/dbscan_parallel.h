//
// Created by xcy on 2020/9/19.
//

#ifndef SRC_DBSCAN_PARALLEL_H
#define SRC_DBSCAN_PARALLEL_H




#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include <thread>

using namespace std;

class ParallelDbscanDriver{
private:
    double EPS ;
    int MinPts ;

public:
    void dbscanClustering(sensor_msgs::PointCloud& PCL);
    static void subThread(int i);
    void mainThread();


    sensor_msgs::PointCloud PCLforOutput;
    void setEPSandMinPts(double eps, int minpts){EPS = eps; MinPts = minpts;}

    enum {visited = 0 , type = 1, cluster = 2};
    enum {core = 0, border = 1, noise = 2, little = 3, strange = 4, pass = 5};

};

#endif //SRC_DBSCAN_PARALLEL_H
