//
// Created by xcy on 2020/5/27.
//

#ifndef SRC_DBSCAN_CLUSTERING_H
#define SRC_DBSCAN_CLUSTERING_H


#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include "octree_nn.h"

using namespace std;

class DBscanDriver{
private:
    OctreeDriver oldDriver;
    double EPS ;
    int MinPts ;
    void clusterVector(vector<vector<double>> currentVector, vector<int> currentIndex, double currentCluster, sensor_msgs::PointCloud &PCL);
public:
    void dbscanClustering(sensor_msgs::PointCloud& PCL);
    sensor_msgs::PointCloud PCLforOutput;
    void setEPSandMinPts(double eps, int minpts){EPS = eps; MinPts = minpts;}

    enum {visited = 0 , type = 1, cluster = 2};
    enum {core = 0, border = 1, noise = 2, little = 3, strange = 4};

};


#endif //SRC_DBSCAN_CLUSTERING_H
