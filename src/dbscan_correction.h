//
// Created by xcy on 2020/8/31.
//

#ifndef SRC_DBSCAN_CORRECTION_H
#define SRC_DBSCAN_CORRECTION_H


#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>

using namespace std;

class NewDbscanDriver{
private:
    double EPS ;
    int MinPts ;
    void clusterVector(vector<vector<double>> currentVector, vector<int> currentIndex,
                       double currentCluster, sensor_msgs::PointCloud &PCL);
    void ellipseSearch(vector<vector<double>>& currentResult, vector<int>& currentIndex,
                       geometry_msgs::Point32 queryPts);
    vector<double> P2stdV(geometry_msgs::Point32 point){
        vector<double> vector;
        vector.resize(3);
        vector[0] = point.x;
        vector[1] = point.y;
        vector[2] = point.z;
        return vector;
    }
    geometry_msgs::Point32 stdV2P(vector<double> vector){
        geometry_msgs::Point32 point;
        point.x = vector[0];
        point.y = vector[1];
        point.z = vector[2];
        return point;
    }
public:
    void dbscanClustering(sensor_msgs::PointCloud& PCL);
    sensor_msgs::PointCloud PCLforOutput;
    void setEPSandMinPts(double eps, int minpts){EPS = eps; MinPts = minpts;}

    enum {visited = 0 , type = 1, cluster = 2};
    enum {core = 0, border = 1, noise = 2, little = 3, strange = 4, pass = 5};

};

#endif //SRC_DBSCAN_CORRECTION_H
