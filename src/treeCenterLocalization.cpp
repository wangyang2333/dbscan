//
// Created by xcy on 2020/8/10.
//

#include "treeCenterLocalization.h"

void TreeCenterLocalization::tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL){
    //if(first track) build map
    if(firstTrackFlag){
        map_cloud.points = landmarkPCL->points;
        firstTrackFlag = false;
    }
        //(publish tf from map to velodyne (zero))
        //(from velodyne to map (zero transform))
    //else: track with current map
        //Hungary Algorithm assignment
        //Ceres optimization Localization (pose of velodyne in map )
        //Publish TF from velodyne to map
        //To Edit Map (landmark form velodyne to map)只能手算吗？

}