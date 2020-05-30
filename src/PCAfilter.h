//
// Created by xcy on 2020/5/30.
//

#ifndef SRC_PCAFILTER_H
#define SRC_PCAFILTER_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
void PCA_Eigen(sensor_msgs::PointCloud& PCL);

void Voxel_Filter_Hash(sensor_msgs::PointCloud& PCL);


#endif //SRC_PCAFILTER_H
