//
// Created by xcy on 2020/8/10.
//

#include "treeCenterLocalization.h"

void TreeCenterLocalization::tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL){
    //if(first track) build map
    if(firstTrackFlag){
        map_cloud.points = landmarkPCL->points;
        firstTrackFlag = false;
        //Read the static TF
//        listener.waitForTransform(base_link_name, lidar_name, ros::Time(0), ros::Duration(3.0));
//        listener.lookupTransform(base_link_name, lidar_name, ros::Time(0), velodyne_to_base);
        //(publish tf from velodyne to map (zero transform))
        velodyne_to_map.setIdentity();
        my_br.sendTransform(tf::StampedTransform(velodyne_to_map, ros::Time::now(), map_name, lidar_name));
    }else{
        /*Track with current map*/
        //Transform datatype to use PCL LIB
        pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_obsCloud (new pcl::PointCloud<pcl::PointXYZ>);

        sensor_msgs::PointCloud2 ROS_PCL2_temp;
        pcl::PCLPointCloud2 PCL_PCL2_temp;
        sensor_msgs::convertPointCloudToPointCloud2(map_cloud, ROS_PCL2_temp);
        pcl_conversions::toPCL(ROS_PCL2_temp, PCL_PCL2_temp);
        pcl::fromPCLPointCloud2(PCL_PCL2_temp, *PCL_mapCloud);

        sensor_msgs::convertPointCloudToPointCloud2(*landmarkPCL, ROS_PCL2_temp);
        pcl_conversions::toPCL(ROS_PCL2_temp, PCL_PCL2_temp);
        pcl::fromPCLPointCloud2(PCL_PCL2_temp, *PCL_obsCloud);

        //Configure and run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputCloud (PCL_obsCloud);
        icp.setInputTarget (PCL_mapCloud);

        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        std::cout << "has converged: " << icp.hasConverged() <<std::endl;
        std::cout << "score: " <<icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        //Publish TF from velodyne to map
        tf::Matrix3x3 tempMat3x3;
        tf::Vector3  tempVec3;
        tf::Quaternion tempQ;
        Eigen::Matrix4d transformation = icp.getFinalTransformation ().cast<double>();
        Eigen::Matrix3d tempRotation = transformation.block<3,3>(0,0);//In eigen type Must be equal!!!!
        Eigen::Vector3d tempTranslation = transformation.block<3,1>(0,3);
        tf::matrixEigenToTF(tempRotation, tempMat3x3);
        tf::vectorEigenToTF(tempTranslation, tempVec3);
        velodyne_to_map.setOrigin(tempVec3);
        tempMat3x3.getRotation(tempQ);
        velodyne_to_map.setRotation(tempQ);
        my_br.sendTransform(tf::StampedTransform(velodyne_to_map, ros::Time::now(), map_name, lidar_name));
        //To Edit Map (landmark form velodyne to map)只能手算吗？
    }
}