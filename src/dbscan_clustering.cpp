//
// Created by xcy on 2020/5/27.
//

#include "dbscan_clustering.h"

void DBscanDriver::clusterVector(vector<vector<double>> currentVector, vector<int> currentIndex, double currentCluster, sensor_msgs::PointCloud &PCL){
    for(int i = 0; i < currentVector.size(); i++){
        /*Skip the visited point*/
        if(PCL.channels[visited].values[currentIndex[i]] == 1.0){
            continue;
        }
        /*Do radiusNN on Octree*/
        oldDriver.searchOctreeRadiusNN(currentVector[i],PCL, oldDriver.root, EPS);
        /*注意复制递归代码时出现的变量名称重复问题*/
        vector<vector<double>> nextResult = oldDriver.getResultVector();
        vector<int> nextIndex = oldDriver.getResultIndex();
        oldDriver.clearResult();
        /*Decide the type*/
        if(nextResult.size() > MinPts){
            PCL.channels[visited].values[currentIndex[i]] = 1.0;
            PCL.channels[type].values[currentIndex[i]] = core;
            PCL.channels[cluster].values[currentIndex[i]] = currentCluster;
            clusterVector(nextResult, nextIndex, currentCluster, PCL);
        }else{
            PCL.channels[visited].values[currentIndex[i]] = 1.0;
            PCL.channels[cluster].values[currentIndex[i]] = currentCluster;
            PCL.channels[type].values[currentIndex[i]] = border;
        }
    }
    return;
}

void DBscanDriver::dbscanClustering(sensor_msgs::PointCloud &PCL) {
    /*Open PCL channels*/
    PCLforOutput = PCL;
    PCLforOutput.channels.clear();
    PCLforOutput.channels.resize(3);
    PCLforOutput.channels[visited].name = "visited";
    PCLforOutput.channels[visited].values.resize(PCLforOutput.points.size(),0.0);
    PCLforOutput.channels[type].name = "type";
    PCLforOutput.channels[type].values.resize(PCLforOutput.points.size());
    PCLforOutput.channels[cluster].name = "cluster";
    PCLforOutput.channels[cluster].values.resize(PCLforOutput.points.size());
    /*Construct the Octree*/
    oldDriver.octreeConstructFromPCL(PCLforOutput);
    int clusterNumCounter = 1;//Cluster 0 means noise, so begin from 1;
    for(int i = 0; i < PCLforOutput.points.size(); i++){
        /*Skip the visited point*/
        if(PCLforOutput.channels[visited].values[i] == 1.0){
            continue;
        }
        /*Do radiusNN on Octree*/
        oldDriver.searchOctreeRadiusNN(oldDriver.P2stdV(PCLforOutput.points[i]), PCLforOutput, oldDriver.root, EPS);
        vector<vector<double>> currentResult = oldDriver.getResultVector();
        vector<int> currentIndex = oldDriver.getResultIndex();
        oldDriver.clearResult();

        //ROS_ERROR("size:%d",(int)currentResult.size());
        /*Decide the type*/
        if(currentResult.size() > MinPts){
            PCLforOutput.channels[visited].values[i] = 1.0;
            PCLforOutput.channels[type].values[i] = core;
            PCLforOutput.channels[cluster].values[i] = clusterNumCounter;
            clusterVector(currentResult, currentIndex, clusterNumCounter, PCLforOutput);
            clusterNumCounter++;
        }else{
            PCLforOutput.channels[visited].values[i] = 1.0;
            PCLforOutput.channels[type].values[i] = noise;
        }
    }
//    for(int i = 0; i < PCLforOutput.points.size(); i++){cout<<PCLforOutput.channels[DBscanDriver::cluster].values[i];}

    return;
}