//
// Created by xcy on 2020/7/15.
//

#include "pass_through.h"
void passThroughTesting(sensor_msgs::PointCloud& dataset, vector<int> Idx){
    /*Initialization to Transform x.y.z to epsilon.alpha.r*/
    sensor_msgs::PointCloud polarDataset = dataset;
    polarDataset.channels.clear();
    polarDataset.channels.resize(3);
    enum {epsilon = 0, alpha = 1, radius = 2};
    polarDataset.channels[epsilon].name = "epsilon";
    polarDataset.channels[epsilon].values.resize(polarDataset.points.size());
    polarDataset.channels[alpha].name = "alpha";
    polarDataset.channels[alpha].values.resize(polarDataset.points.size());
    polarDataset.channels[radius].name = "radius";
    polarDataset.channels[radius].values.resize(polarDataset.points.size());
    /*Compute epsilon.alpha.r*/
    for(int i = 0; i < polarDataset.points.size(); i++){
        double x = dataset.points[i].x;
        double y = dataset.points[i].y;
        double z = dataset.points[i].z;
        polarDataset.channels[radius].values[i] = sqrt(x*x +y*y +z*z);
        polarDataset.channels[epsilon].values[i] = atan2(z,sqrt(x*x+y*y));
        polarDataset.channels[alpha].values[i] = atan2(y,x);
    }
    /*Initialize the Corners*/
    enum {up = 0, down = 1, left = 2, right = 3};
    vector<double> treeCorners;
    treeCorners.resize(4);
    treeCorners[up] = -INFINITY;
    treeCorners[down] = INFINITY;
    treeCorners[left] = -M_PI;
    treeCorners[right] = M_PI;
    //TODO:border at M_PI effect
    /*Compute the four borders*/
    for(int k = 0; k < Idx.size(); k++){
        if(polarDataset.channels[epsilon].values[Idx[k]] > treeCorners[up]){
            treeCorners[up] = polarDataset.channels[epsilon].values[Idx[k]];
        }
        if(polarDataset.channels[epsilon].values[Idx[k]] < treeCorners[down]){
            treeCorners[down] = polarDataset.channels[epsilon].values[Idx[k]];
        }
        if(polarDataset.channels[alpha].values[Idx[k]] > treeCorners[left]){
            treeCorners[left] = polarDataset.channels[alpha].values[Idx[k]];
        }
        if(polarDataset.channels[alpha].values[Idx[k]] < treeCorners[right]){
            treeCorners[right] = polarDataset.channels[alpha].values[Idx[k]];
        }
    }
    /*Test Pass Through*/
    int passThroughNum = 0;
    for(int i = 0; i < polarDataset.points.size(); i++){
        if(polarDataset.channels[epsilon].values[i] < treeCorners[up] &&
        polarDataset.channels[epsilon].values[i] > treeCorners[down] &&
        polarDataset.channels[alpha].values[i] < treeCorners[left] &&
        polarDataset.channels[alpha].values[i] > treeCorners[right]){
            passThroughNum++;
        }
    }

    bool passThrough = false;
    if(passThroughNum > Idx.size()){
        passThrough = true;
        ROS_WARN("Pass Through with %d inner point, %d points in total", passThroughNum, (int)Idx.size() );
    }

    if(passThrough){
        for(int k = 0; k < Idx.size(); k++){
            dataset.channels[NewDbscanDriver::type].values[Idx[k]] = NewDbscanDriver::pass;
            dataset.channels[NewDbscanDriver::cluster].values[Idx[k]] = 0.0;
        }
    }else{

    }
}