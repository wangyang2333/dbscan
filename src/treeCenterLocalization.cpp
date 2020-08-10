//
// Created by xcy on 2020/8/10.
//

#include "treeCenterLocalization.h"

void TreeCenterLocalization::tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL){
    //if(first track) build map
    if(true){
        map_cloud.points = landmarkPCL->points;
        firstTrackFlag = false;
        //Read the static TF
        listener.waitForTransform(base_link_name, lidar_name, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(base_link_name, lidar_name, ros::Time(0), velodyne_to_base);
        //(publish tf from velodyne to map (zero transform))
        velodyne_to_map.setIdentity();
        my_br.sendTransform(tf::StampedTransform(velodyne_to_map, ros::Time::now(), map_name, lidar_name));
    }

    //else: track with current map
        //Hungary Algorithm assignment
        //Ceres optimization Localization (pose of velodyne in map )
        //Publish TF from velodyne to map
        //To Edit Map (landmark form velodyne to map)只能手算吗？

}