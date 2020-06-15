//
// Created by xcy on 2020/6/14.
//

#ifndef SRC_FPFH_PCL_H
#define SRC_FPFH_PCL_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>             // 法线
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>   // 可视化
#include <pcl/visualization/pcl_plotter.h>
#include <chrono>

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

class FPFHDriver{
private:
public:
    void FPFH(sensor_msgs::PointCloud& PCL);

};


#endif //SRC_FPFH_PCL_H
