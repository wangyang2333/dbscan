//
// Created by xcy on 2020/5/18.
//

#ifndef SRC_GMM_EM_H
#define SRC_GMM_EM_H


#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include "kd_tree_nn.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
sensor_msgs::PointCloud gmmEM(sensor_msgs::PointCloud PCL);


#endif //SRC_GMM_EM_H
