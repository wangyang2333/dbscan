//
// Created by xcy on 2020/5/4.
//

#ifndef SRC_OCTREE_NN_H
#define SRC_OCTREE_NN_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>

using namespace std;

//定义八叉树节点类

struct OctreeNode
{
    OctreeNode* children[8];
    vector<double> center;
    double extent;
    vector<int> point_indice;
    bool isLeaf;
    OctreeNode(vector<double> center_, double extent_, vector<int> point_indice_, bool isLeaf_){
        for(int i =0; i< 8; i++){
            children[i] = NULL;
        }
        center = center_;
        extent = extent_;
        point_indice = point_indice_;
        isLeaf = isLeaf_;
    }

};

void OCTREE_NN(sensor_msgs::PointCloud& PCL);


#endif //SRC_OCTREE_NN_H
