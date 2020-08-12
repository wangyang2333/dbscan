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

// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (MaxCorrespondenceDistance);
// Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (MaximumIterations);
// Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (setTransformationEpsilon);
// Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (EuclideanFitnessEpsilon);

//        icp.setRANSACIterations(50);
//        icp.setRANSACOutlierRejectionThreshold(0.05);

        icp.setUseReciprocalCorrespondences (true);

        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        std::cout << "has converged: " << icp.hasConverged() <<std::endl;
        std::cout << "score: " <<icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        if(icp.hasConverged()){
            //Publish TF from velodyne to map
            tf::Matrix3x3 tempMat3x3;
            tf::Vector3  tempVec3;
            tf::Quaternion tempQ;
            Eigen::Matrix4d transformation = icp.getFinalTransformation ().cast<double>();
            Eigen::Matrix3d tempRotation = transformation.block<3,3>(0,0);//In eigen type Must be equal!!!!
            Eigen::Vector3d tempTranslation = transformation.block<3,1>(0,3);
            tf::matrixEigenToTF(tempRotation, tempMat3x3);
            tf::vectorEigenToTF(tempTranslation, tempVec3);
            /////tempMat3x3 = velodyne_to_map.getRotation() * tempMat3x3;
            velodyne_to_map.setOrigin(tempVec3);
            tempMat3x3.getRotation(tempQ);
            velodyne_to_map.setRotation(tempQ);
            my_br.sendTransform(tf::StampedTransform(velodyne_to_map, ros::Time::now(), map_name, lidar_name));
        }else{
            //TODO:
        }
        //To Edit Map (landmark form velodyne to map)
        /*To find new coming point*/
        sensor_msgs::PointCloud ptsToBeAddedToMap = *landmarkPCL;
        ptsToBeAddedToMap.channels.resize(1);
        ptsToBeAddedToMap.channels[0].name = "trackSuccess";
        ptsToBeAddedToMap.channels[0].values.resize(ptsToBeAddedToMap.points.size());


        pcl::Correspondences currentCorrespondences = *icp.correspondences_;
        for(int i = 0; i < currentCorrespondences.size(); i++){
//            cout<<"i:"<< i <<endl;
//            cout<<"index_match:"<<currentCorrespondences[i].index_match<<endl;
//            cout<<"index_query:"<<currentCorrespondences[i].index_query<<endl;
//            cout<<"distance:"<<currentCorrespondences[i].distance<<endl;
//            cout<<"weight:"<<currentCorrespondences[i].weight<<endl;
            ptsToBeAddedToMap.channels[0].values[i] = 1;
        }
        addPointsToMap(ptsToBeAddedToMap, map_cloud);
    }
    landmark_cloud_pub.publish(map_cloud);
}

void TreeCenterLocalization::addPointsToMap(sensor_msgs::PointCloud pointsToBeAdded, sensor_msgs::PointCloud& map) {
    map.points.clear();
    for(int i =0; i < pointsToBeAdded.points.size(); i++){
        if(pointsToBeAdded.channels[0].values[i] != 1){

        }
        addOnePtToMap(pointsToBeAdded.points[i]);
    }


}

geometry_msgs::Point32 TreeCenterLocalization::changeFrame(geometry_msgs::Point32 sourcePoint, string sourceFrame, string targetFrame){
    geometry_msgs::PointStamped sourcePointStamped;
    sourcePointStamped.header.frame_id = sourceFrame;
    sourcePointStamped.header.stamp = ros::Time();

    sourcePointStamped.point.x = sourcePoint.x;
    sourcePointStamped.point.y = sourcePoint.y;
    sourcePointStamped.point.z = sourcePoint.z;
    geometry_msgs::PointStamped targetPointStamped;

    listener.waitForTransform(sourceFrame, targetFrame, ros::Time(0), ros::Duration(3.0));
    listener.transformPoint(targetFrame, sourcePointStamped, targetPointStamped);

    geometry_msgs::Point32 pt_out;
    pt_out.x = targetPointStamped.point.x;
    pt_out.y = targetPointStamped.point.y;
    pt_out.z = targetPointStamped.point.z;
    return pt_out;
}

void TreeCenterLocalization::addOnePtToMap(geometry_msgs::Point32 new_landmark){
    //add new tree to map_cloud
    geometry_msgs::Point32 finalPt;
    finalPt = changeFrame(new_landmark,lidar_name,map_name);
    map_cloud.points.push_back(finalPt);
    //TreeId------map_cloud.channels[0].values.push_back(new_landmark);

}