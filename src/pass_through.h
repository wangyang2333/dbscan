//
// Created by xcy on 2020/7/15.
//

#ifndef SRC_PASS_THROUGH_H
#define SRC_PASS_THROUGH_H


#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "dbscan_clustering.h"
using namespace std;
void passThroughTesting(sensor_msgs::PointCloud& PCL, vector<int> Idx);

#endif //SRC_PASS_THROUGH_H
