//
// Created by xcy on 2020/8/31.
//

#include "dbscan_correction.h"
void NewDbscanDriver::clusterVector(vector<vector<double>> currentVector, vector<int> currentIndex,
                                    double currentCluster, sensor_msgs::PointCloud &PCL){
    for(int i = 0; i < currentVector.size(); i++){
        /*Skip the visited point*/
        if(PCL.channels[visited].values[currentIndex[i]] == 1.0){
            continue;
        }
        /*Do Brute Force Ellipse Search around currentVector[i]*/
        vector<vector<double>> nextResult;
        vector<int> nextIndex;
        ellipseSearch(nextResult, nextIndex, stdV2P(currentVector[i]));
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

void NewDbscanDriver::dbscanClustering(sensor_msgs::PointCloud &PCL) {
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
    /*Loop Over All Points*/
    int clusterNumCounter = 1;//Cluster 0 means noise, so begin from 1;
    for(int i = 0; i < PCLforOutput.points.size(); i++){
        /*Skip the visited point*/
        if(PCLforOutput.channels[visited].values[i] == 1.0){
            continue;
        }

        /*Do Brute Force Ellipse Search around point PCLforOutput.channels[visited].values[i]*/
        vector<vector<double>> currentResult;
        vector<int> currentIndex;
        ellipseSearch(currentResult, currentIndex, PCLforOutput.points[i]);

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
//    for(int i = 0; i < PCLforOutput.points.size(); i++){cout<<PCLforOutput.channels[NewDbscanDriver::cluster].values[i];}

    return;
}

void NewDbscanDriver::ellipseSearch(vector<vector<double>>& currentResult, vector<int>& currentIndex,
                                    geometry_msgs::Point32 queryPts) {
    double fai = 2.0 / 180.0 * M_PI;
    double delta = 0.2 / 180.0 * M_PI;

    double sitaz = atan2(queryPts.z, sqrt(queryPts.x*queryPts.x + queryPts.y*queryPts.y));
    double a, b, c;
    c = (sqrt(queryPts.x*queryPts.x + queryPts.y*queryPts.y) * (tan(fai + sitaz) - tan(fai - sitaz)))/2.0 * EPS;
    a = b = (sqrt(queryPts.x*queryPts.x + queryPts.y*queryPts.y) * delta)* EPS;
    for(int i = 0; i < PCLforOutput.points.size(); i++){
        double leftHandSide = (PCLforOutput.points[i].x - queryPts.x) * (PCLforOutput.points[i].x - queryPts.x) / a / a +
                (PCLforOutput.points[i].y - queryPts.y) * (PCLforOutput.points[i].y - queryPts.y) / b / b +
                (PCLforOutput.points[i].z - queryPts.z) * (PCLforOutput.points[i].z - queryPts.z) / c / c ;
        if(leftHandSide < 1.0){
            currentIndex.push_back(i);
            currentResult.push_back(P2stdV(PCLforOutput.points[i]));
        }
    }
}