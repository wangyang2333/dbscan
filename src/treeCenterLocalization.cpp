//
// Created by xcy on 2020/8/10.
//

#include "treeCenterLocalization.h"

void TreeCenterLocalization::tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL){
    //if(first track) build map
    if(firstTrackFlag){
        ROS_WARN("First Track, build map with all points.");
        myAtlas.atlasIntializationWithPCL(*landmarkPCL, map_name);

        firstTrackFlag = false;
        //Read the static TF
//        listener.waitForTransform(base_link_name, lidar_name, ros::Time(0), ros::Duration(3.0));
//        listener.lookupTransform(base_link_name, lidar_name, ros::Time(0), velodyne_to_base);
        //(publish tf from velodyne to map (zero transform))
        velodyne_to_map.setIdentity();
        my_br.sendTransform(tf::StampedTransform(velodyne_to_map, ros::Time::now(), map_name, lidar_name));

        my_pose.header.frame_id = map_name;
        my_pose.header.stamp = ros::Time::now();
        my_pose.pose.position.x = velodyne_to_map.getOrigin().getX();
        my_pose.pose.position.y = velodyne_to_map.getOrigin().getY();
        my_pose.pose.position.z = 0;
        tf::quaternionTFToMsg(velodyne_to_map.getRotation(), my_pose.pose.orientation);
        my_pose_publisher.publish(my_pose);
    }else{
        /*Track with current map*/
        //Transform datatype to use PCL LIB
        pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_obsCloud (new pcl::PointCloud<pcl::PointXYZ>);

        sensor_msgs::PointCloud2 ROS_PCL2_temp;
        pcl::PCLPointCloud2 PCL_PCL2_temp;

        sensor_msgs::convertPointCloudToPointCloud2(myAtlas.getLocalMapWithTF(velodyne_to_map), ROS_PCL2_temp);
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
        icp.align(Final, initialGuessOfICP);

        std::cout << "has converged: " << icp.hasConverged() <<std::endl;
        std::cout << "score: " <<icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        if(icp.hasConverged()){
            //Publish TF from velodyne to map
            tf::Matrix3x3 tempMat3x3;
            tf::Vector3  tempVec3;
            tf::Quaternion tempQ;
            Eigen::Matrix4d transformation = icp.getFinalTransformation ().cast<double>();
            initialGuessOfICP = icp.getFinalTransformation ();
            Eigen::Matrix3d tempRotation = transformation.block<3,3>(0,0);//In eigen type Must be equal!!!!
            Eigen::Vector3d tempTranslation = transformation.block<3,1>(0,3);
            tf::matrixEigenToTF(tempRotation, tempMat3x3);
            tf::vectorEigenToTF(tempTranslation, tempVec3);
            /////tempMat3x3 = velodyne_to_map.getRotation() * tempMat3x3;
            velodyne_to_map.setOrigin(tempVec3);
            tempMat3x3.getRotation(tempQ);
            velodyne_to_map.setRotation(tempQ);
            my_br.sendTransform(tf::StampedTransform(velodyne_to_map, ros::Time::now(), map_name, lidar_name));

            my_pose.header.frame_id = map_name;
            my_pose.header.stamp = ros::Time::now();
            my_pose.pose.position.x = velodyne_to_map.getOrigin().getX();
            my_pose.pose.position.y = velodyne_to_map.getOrigin().getY();
            my_pose.pose.position.z = 0;
            tf::quaternionTFToMsg(velodyne_to_map.getRotation(), my_pose.pose.orientation);
            my_pose_publisher.publish(my_pose);
        }else{
            ROS_ERROR("No convergence!");
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
        myAtlas.addPointsToMapWithTF(ptsToBeAddedToMap, velodyne_to_map);
    }
    landmark_cloud_pub.publish(myAtlas.getFullAtlas());
}

geometry_msgs::Point32 TreeCenterLocalization::changeFrame(geometry_msgs::Point32 sourcePoint, string sourceFrame, string targetFrame){
    geometry_msgs::PointStamped sourcePointStamped;
    sourcePointStamped.header.frame_id = sourceFrame;
    sourcePointStamped.header.stamp = ros::Time();

    sourcePointStamped.point.x = sourcePoint.x;
    sourcePointStamped.point.y = sourcePoint.y;
    sourcePointStamped.point.z = sourcePoint.z;
    geometry_msgs::PointStamped targetPointStamped;

    //listener.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(3.0));
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
    myAtlas.fullLandMarks.points.push_back(finalPt);
    //TreeId------map_cloud.channels[0].values.push_back(new_landmark);

}



void TreeAtlas::realTimeTransformPointCloud(const std::string & target_frame, const tf::Transform& net_transform,
                                            const ros::Time& target_time, const sensor_msgs::PointCloud & cloudIn,
                                            sensor_msgs::PointCloud & cloudOut) const
{
    tf::Vector3 origin = net_transform.getOrigin();
    tf::Matrix3x3 basis  = net_transform.getBasis();

    unsigned int length = cloudIn.points.size();

    // Copy relevant data from cloudIn, if needed
    if (&cloudIn != &cloudOut)
    {
        cloudOut.header = cloudIn.header;
        cloudOut.points.resize(length);
        cloudOut.channels.resize(cloudIn.channels.size());
        for (unsigned int i = 0 ; i < cloudIn.channels.size() ; ++i)
            cloudOut.channels[i] = cloudIn.channels[i];
    }

    // Transform points
    cloudOut.header.stamp = target_time;
    cloudOut.header.frame_id = target_frame;
    for (unsigned int i = 0; i < length ; i++) {
        double x = basis[0].x() * cloudIn.points[i].x + basis[0].y() * cloudIn.points[i].y + basis[0].z() * cloudIn.points[i].z + origin.x();
        double y = basis[1].x() * cloudIn.points[i].x + basis[1].y() * cloudIn.points[i].y + basis[1].z() * cloudIn.points[i].z + origin.y();
        double z = basis[2].x() * cloudIn.points[i].x + basis[2].y() * cloudIn.points[i].y + basis[2].z() * cloudIn.points[i].z + origin.z();
        cloudOut.points[i].x = x; cloudOut.points[i].y = y; cloudOut.points[i].z = z;
    }
}


void TreeAtlas::addPointsToMapWithTF(sensor_msgs::PointCloud pointsToBeAdded, tf::StampedTransform currentTF) {
    fullLandMarks.points.clear();
//    for(int i =0; i < pointsToBeAdded.points.size(); i++){
//        if(pointsToBeAdded.channels[0].values[i] != 1){
//
//        }
//        addOnePtToMap(pointsToBeAdded.points[i]);
//    }
    sensor_msgs::PointCloud temp_map;
    realTimeTransformPointCloud(map_name, currentTF, fullLandMarks.header.stamp, pointsToBeAdded, temp_map);
    fullLandMarks = temp_map;

    //listener.transformPointCloud(map_name, pointsToBeAdded, map);
}

sensor_msgs::PointCloud TreeAtlas::getLocalMapWithTF(tf::StampedTransform currentTF){
    return fullLandMarks;
}