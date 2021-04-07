//
// Created by xcy on 2020/8/10.
//

#include "treeCenterLocalization.h"

#include <utility>

void TreeCenterLocalization::tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL){
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    //if(first track) build map
    if(firstTrackFlag){
        ROS_WARN("First Track, build map with all points.");
        myAtlas.atlasIntializationWithPCL(*landmarkPCL, map_name);
        firstTrackFlag = false;

        velodyne_to_map.setIdentity();
        my_br.sendTransform(tf::StampedTransform(velodyne_to_map, landmarkPCL->header.stamp, map_name, lidar_name));

        my_pose.header.frame_id = map_name;
        my_pose.header.stamp = landmarkPCL->header.stamp;
        my_pose.pose.position.x = velodyne_to_map.getOrigin().getX();
        my_pose.pose.position.y = velodyne_to_map.getOrigin().getY();
        my_pose.pose.position.z = 0;
        tf::quaternionTFToMsg(velodyne_to_map.getRotation(), my_pose.pose.orientation);
        my_pose_publisher.publish(my_pose);
        lastPoseOfICP.setIdentity();
        initialGuessOfICP.setIdentity();
    }else{
        if(ICPwithStableMap(landmarkPCL)){
            ICPwithfullLandmarks(landmarkPCL);
        }
    }
    landmark_cloud_pub.publish(myAtlas.getFullAtlas());
}

void TreeCenterLocalization::posePredict(Eigen::Matrix<float, 4, 4> tfA,
                                                             Eigen::Matrix<float, 4, 4> tfB,
                                                             Eigen::Matrix<float, 4, 4>& result) {
    result.block<3,3>(0,0) = tfB.block<3,3>(0,0).inverse();
    result.block<3,1>(0,3) = -1*tfB.block<3,3>(0,0).inverse() * tfB.block<3,1>(0,3);
    result.block<1,3>(3,0).setZero();
    result.block<1,1>(3,3).setConstant(1);
    /*This prediction seems to cause divergence*/
    //result = tfA * result * tfA;
    result = tfA;
    result.block<1,1>(3,2).setConstant(0);
}

bool TreeCenterLocalization::ICPwithStableMap(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL) {
    clock_t startTime,endTime;
    startTime = clock();//计时开始
    //Transform datatype to use PCL LIB
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_obsCloud (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 ROS_PCL2_temp;
    pcl::PCLPointCloud2 PCL_PCL2_temp;
    sensor_msgs::convertPointCloudToPointCloud2(myAtlas.getStableMapWithTF(velodyne_to_map), ROS_PCL2_temp);
    pcl_conversions::toPCL(ROS_PCL2_temp, PCL_PCL2_temp);
    pcl::fromPCLPointCloud2(PCL_PCL2_temp, *PCL_mapCloud);

    sensor_msgs::convertPointCloudToPointCloud2(*landmarkPCL, ROS_PCL2_temp);
    pcl_conversions::toPCL(ROS_PCL2_temp, PCL_PCL2_temp);
    pcl::fromPCLPointCloud2(PCL_PCL2_temp, *PCL_obsCloud);

    //Configure and run ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(PCL_obsCloud);
    icp.setInputTarget(PCL_mapCloud);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (MaxCorrespondenceDistance);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (int(MaximumIterations));
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (setTransformationEpsilon);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (EuclideanFitnessEpsilon);

//        icp.setRANSACIterations(100);
//        icp.setRANSACOutlierRejectionThreshold(0.5);

    icp.setUseReciprocalCorrespondences (true);

    pcl::PointCloud<pcl::PointXYZ> Final;


    icp.align(Final, initialGuessOfICP);

    pcl::Correspondences currentCorrespondences = *icp.correspondences_;
    if(icp.hasConverged()){
        /*Use a constant velocity to generate an initial guess*/
        posePredict(icp.getFinalTransformation(), lastPoseOfICP, initialGuessOfICP);
        lastPoseOfICP = icp.getFinalTransformation();

        //Publish TF from velodyne to map
        tf::Matrix3x3 tempMat3x3;
        tf::Vector3  tempVec3;
        tf::Quaternion tempQ;
        Eigen::Matrix4d transformation = icp.getFinalTransformation ().cast<double>();
        lastPoseOfICP = icp.getFinalTransformation ();

        Eigen::Matrix3d tempRotation = transformation.block<3,3>(0,0);//In eigen type Must be equal!!!!
        Eigen::Vector3d tempTranslation = transformation.block<3,1>(0,3);
        tf::matrixEigenToTF(tempRotation, tempMat3x3);
        tf::vectorEigenToTF(tempTranslation, tempVec3);
        velodyne_to_map.setOrigin(tempVec3);
        tempMat3x3.getRotation(tempQ);
        velodyne_to_map.setRotation(tempQ);
        my_br.sendTransform(tf::StampedTransform(velodyne_to_map, landmarkPCL->header.stamp, map_name, lidar_name));
        /*Publish pose*/
        my_pose.header.frame_id = map_name;
        my_pose.header.stamp = landmarkPCL->header.stamp;
        my_pose.pose.position.x = velodyne_to_map.getOrigin().getX();
        my_pose.pose.position.y = velodyne_to_map.getOrigin().getY();
        my_pose.pose.position.z = 0;
        tf::quaternionTFToMsg(velodyne_to_map.getRotation(), my_pose.pose.orientation);
        my_pose_publisher.publish(my_pose);
        /*Publish Odometry*/
        my_odometry.pose.pose = my_pose.pose;
        my_odometry.header = my_pose.header;
        my_odometry_publisher.publish(my_odometry);
        /*Publish Trajectory in Path msg*/


        robotPath.header.stamp = landmarkPCL->header.stamp;
        robotPath.header.frame_id = map_name;
        robotPath.poses.push_back(my_pose);
        path_pub.publish(robotPath);
    }else{
        ROS_ERROR("No convergence In Stable ICP!");
        for(int i = 0; i < currentCorrespondences.size(); i++){
            cout<<"localMapsize: "<<myAtlas.getStableMap().points.size()<<endl;
            cout<<"scanSize: "<<landmarkPCL->points.size()<<endl;
            cout<<"i:"<< i <<endl;
            cout<<"index_match:"<<currentCorrespondences[i].index_match<<endl;
            cout<<"index_query:"<<currentCorrespondences[i].index_query<<endl;
            cout<<"distance:"<<currentCorrespondences[i].distance<<endl;
            cout<<"weight:"<<currentCorrespondences[i].weight<<endl;
        }
        ROS_ERROR("No convergence In full ICP!");
    }
    endTime = clock();//计时结束
    cout << "The stable ICP run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return icp.hasConverged();
}
bool TreeCenterLocalization::ICPwithfullLandmarks(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL) {
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    /*Track with current map*/
    sensor_msgs::PointCloud localMap = myAtlas.getLocalMapWithTF(velodyne_to_map);

    //Transform datatype to use PCL LIB
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_obsCloud (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 ROS_PCL2_temp;
    pcl::PCLPointCloud2 PCL_PCL2_temp;

    sensor_msgs::convertPointCloudToPointCloud2(localMap, ROS_PCL2_temp);
    pcl_conversions::toPCL(ROS_PCL2_temp, PCL_PCL2_temp);
    pcl::fromPCLPointCloud2(PCL_PCL2_temp, *PCL_mapCloud);

    sensor_msgs::convertPointCloudToPointCloud2(*landmarkPCL, ROS_PCL2_temp);
    pcl_conversions::toPCL(ROS_PCL2_temp, PCL_PCL2_temp);
    pcl::fromPCLPointCloud2(PCL_PCL2_temp, *PCL_obsCloud);

    //Configure and run ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(PCL_obsCloud);
    icp.setInputTarget (PCL_mapCloud);


    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (MaxCorrespondenceDistance);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (int(MaximumIterations));
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (setTransformationEpsilon);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (EuclideanFitnessEpsilon);

//        icp.setRANSACIterations(100);
//        icp.setRANSACOutlierRejectionThreshold(0.5);

    icp.setUseReciprocalCorrespondences (true);

    pcl::PointCloud<pcl::PointXYZ> Final;
    /*need a initial value from stable ICP*/
    icp.align(Final, lastPoseOfICP);
    pcl::Correspondences currentCorrespondences = *icp.correspondences_;

    if(icp.hasConverged()){
        /*Do not update the pose here*/
    }else{
        for(int i = 0; i < currentCorrespondences.size(); i++){
            cout<<"localMapsize: "<<localMap.points.size()<<endl;
            cout<<"scanSize: "<<landmarkPCL->points.size()<<endl;
            cout<<"i:"<< i <<endl;
            cout<<"index_match:"<<currentCorrespondences[i].index_match<<endl;
            cout<<"index_query:"<<currentCorrespondences[i].index_query<<endl;
            cout<<"distance:"<<currentCorrespondences[i].distance<<endl;
            cout<<"weight:"<<currentCorrespondences[i].weight<<endl;
        }
        ROS_ERROR("No convergence In full ICP!");
    }
    //To Edit Map (landmark form velodyne to map)
    /*To find new coming point*/
    sensor_msgs::PointCloud ptsToBeAddedToMap = *landmarkPCL;
    ptsToBeAddedToMap.channels.resize(4);
    ptsToBeAddedToMap.channels[TrackSuccess].name = "trackSuccess";
    ptsToBeAddedToMap.channels[TrackSuccess].values.resize(ptsToBeAddedToMap.points.size());
    ptsToBeAddedToMap.channels[IdxInFullMap].name = "IdxInFullMap";
    ptsToBeAddedToMap.channels[IdxInFullMap].values.resize(ptsToBeAddedToMap.points.size());

    for(auto & currentCorrespondence : currentCorrespondences){
        ptsToBeAddedToMap.channels[TrackSuccess].values[currentCorrespondence.index_query] = 1;
        ptsToBeAddedToMap.channels[IdxInFullMap].values[currentCorrespondence.index_query] =
                localMap.channels[IdxInFullMap].values[currentCorrespondence.index_match];
    }
    myAtlas.addPointsToMapWithTF(ptsToBeAddedToMap, velodyne_to_map);
    endTime = clock();//计时结束
    cout << "The full ICP run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return icp.hasConverged();
}

geometry_msgs::Point32 TreeCenterLocalization::changeFrame(geometry_msgs::Point32 sourcePoint, string sourceFrame,
                                                           const string& targetFrame){
    geometry_msgs::PointStamped sourcePointStamped;
    sourcePointStamped.header.frame_id = std::move(sourceFrame);

    sourcePointStamped.point.x = sourcePoint.x;
    sourcePointStamped.point.y = sourcePoint.y;
    sourcePointStamped.point.z = sourcePoint.z;
    geometry_msgs::PointStamped targetPointStamped;

    listener.transformPoint(targetFrame, sourcePointStamped, targetPointStamped);

    geometry_msgs::Point32 pt_out;
    pt_out.x = targetPointStamped.point.x;
    pt_out.y = targetPointStamped.point.y;
    pt_out.z = targetPointStamped.point.z;
    return pt_out;
}


void TreeAtlas::realTimeTransformPointCloud(const std::string & target_frame, const tf::Transform& net_transform,
                                            const ros::Time& target_time, const sensor_msgs::PointCloud & cloudIn,
                                            sensor_msgs::PointCloud & cloudOut)
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

void TreeAtlas::addPointsToMapWithTF(sensor_msgs::PointCloud pointsToBeAdded, const tf::StampedTransform& currentTF) {
    //fullLandMarks.points.clear();
    /*If tracked? erase it. All new point coming points left now*/
    for(int i =0; i < pointsToBeAdded.points.size(); i++){
        /*if New point*/
        if(pointsToBeAdded.channels[TrackSuccess].values[i] == 1){
            //
            pointsToBeAdded.points.erase(i+pointsToBeAdded.points.begin());
            fullLandMarks.channels[TrackingTimes].values[pointsToBeAdded.channels[IdxInFullMap].values[i]]++;
            i--;
        }
    }

    sensor_msgs::PointCloud temp_map;
    realTimeTransformPointCloud(map_name, currentTF, pointsToBeAdded.header.stamp, pointsToBeAdded, temp_map);
    temp_map.channels.clear();
    temp_map.channels.resize(4);
    temp_map.channels[BirthTime].name = "BirthTime";
    temp_map.channels[TrackingTimes].name = "TrackingTimes";
    temp_map.channels[BirthTime].values.resize(temp_map.points.size(), pointsToBeAdded.header.stamp.toSec()-initialTime);
    temp_map.channels[TrackingTimes].values.resize(temp_map.points.size(),0);
    fullLandMarks.points.insert(fullLandMarks.points.end(), temp_map.points.begin(), temp_map.points.end());
    fullLandMarks.channels[BirthTime].values.insert(fullLandMarks.channels[BirthTime].values.end(),
                        temp_map.channels[BirthTime].values.begin(), temp_map.channels[BirthTime].values.end());
    fullLandMarks.channels[TrackingTimes].values.insert(fullLandMarks.channels[TrackingTimes].values.end(),
                        temp_map.channels[TrackingTimes].values.begin(), temp_map.channels[TrackingTimes].values.end());

    /*Eliminate the Points with long time and Low tracking time.*/
    double currentTime = pointsToBeAdded.header.stamp.toSec()-initialTime;
    for(int i = 0; i < fullLandMarks.points.size(); i ++){
        if(currentTime - fullLandMarks.channels[BirthTime].values[i] > birthTimeThreshould &&
        fullLandMarks.channels[TrackingTimes].values[i] < TrackingTimesThreshould && currentTime > removalBeginTime){
            //ROS_WARN("ERASE");
            fullLandMarks.points.erase(fullLandMarks.points.begin() + i);
            for(auto & channel : fullLandMarks.channels){
                if(channel.values.size() == fullLandMarks.points.size() + 1)
                channel.values.erase(i + channel.values.begin());
            }
        }
    }
    if( currentTime < stableTimeThreshould){
        stableMap.points = fullLandMarks.points;
        return;
    }
    stableMap.points.clear();
    for(int i = 0; i < fullLandMarks.points.size(); i ++){
        if(fullLandMarks.channels[TrackingTimes].values[i] > stableTrackingThreshould){
//            /*If this point has a old famous neighbor, erase it*/
//            bool duplicated = false;
//            for(int j = 0; j < fullLandMarks.points.size(); j++){
//                if(((pointsToBeAdded.points[i].x - fullLandMarks.points[j].x)*(pointsToBeAdded.points[i].x - fullLandMarks.points[j].x)+
//                    (pointsToBeAdded.points[i].y - fullLandMarks.points[j].y)*(pointsToBeAdded.points[i].y - fullLandMarks.points[j].y)+
//                    (pointsToBeAdded.points[i].z - fullLandMarks.points[j].z)*(pointsToBeAdded.points[i].z - fullLandMarks.points[j].z)) < 1){
//
//                }
//            }
            stableMap.points.push_back(fullLandMarks.points[i]);
        }
    }
}

/*get Radius NN PCL on Octree*/
/*Find Landmarks point with it's index*/
sensor_msgs::PointCloud TreeAtlas::getLocalMapWithTF(tf::StampedTransform currentTF){
    localMap.points.clear();
    localMap.channels[IdxInFullMap].values.clear();
    /*Build Octree with full landmarks*/
    oldDriver.octreeConstructFromPCL(fullLandMarks);
    /*Transform TF to vector*/
    vector<double> vec;
    vec.resize(3);
    vec[0] = currentTF.getOrigin().getX();
    vec[1] = currentTF.getOrigin().getY();
    vec[2] = currentTF.getOrigin().getZ();
    /*Do radiusNN on Octree*/
    oldDriver.searchOctreeRadiusNN(vec, fullLandMarks, oldDriver.root, localMapRadius);
    vector<vector<double>> currentResult = oldDriver.getResultVector();
    vector<int> currentIndex = oldDriver.getResultIndex();
    oldDriver.clearResult();
    /*Pack Idx and Vec to PCL*/
    for(int i = 0; i < currentResult.size(); i++){
        geometry_msgs::Point32 tempPt;
        tempPt.x = currentResult[i][0];
        tempPt.y = currentResult[i][1];
        tempPt.z = currentResult[i][2];
        localMap.points.push_back(tempPt);
        localMap.channels[IdxInFullMap].values.push_back(currentIndex[i]);
    }

    return localMap;
}

sensor_msgs::PointCloud TreeAtlas::getStableMapWithTF(tf::StampedTransform currentTF) {
    sensor_msgs::PointCloud localStableMap = stableMap;
    localStableMap.points.clear();
    /*Build Octree with full landmarks*/
    oldDriver.octreeConstructFromPCL(stableMap);
    /*Transform TF to vector*/
    vector<double> vec;
    vec.resize(3);
    vec[0] = currentTF.getOrigin().getX();
    vec[1] = currentTF.getOrigin().getY();
    vec[2] = currentTF.getOrigin().getZ();
    /*Do radiusNN on Octree*/
    oldDriver.searchOctreeRadiusNN(vec, stableMap, oldDriver.root, localMapRadius);
    vector<vector<double>> currentResult = oldDriver.getResultVector();
    vector<int> currentIndex = oldDriver.getResultIndex();
    oldDriver.clearResult();
    /*Pack Idx and Vec to PCL*/
    for(int i = 0; i < currentResult.size(); i++){
        geometry_msgs::Point32 tempPt;
        tempPt.x = currentResult[i][0];
        tempPt.y = currentResult[i][1];
        tempPt.z = currentResult[i][2];
        localStableMap.points.push_back(tempPt);
    }
    return localStableMap;
}

void TreeAtlas::atlasIntializationWithPCL(const sensor_msgs::PointCloud& initialPCL, string globalFrame){
    initialTime = initialPCL.header.stamp.toSec();
    map_name = std::move(globalFrame);
    lidar_name = initialPCL.header.frame_id;

    stableMap.header.frame_id = map_name;
    stableMap.points = initialPCL.points;

    fullLandMarks.header.frame_id = map_name;
    fullLandMarks.points = initialPCL.points;
    fullLandMarks.channels.resize(4);
    fullLandMarks.channels[BirthTime].name = "BirthTime";
    fullLandMarks.channels[TrackingTimes].name = "TrackingTimes";
    fullLandMarks.channels[BirthTime].values.resize(fullLandMarks.points.size(), initialPCL.header.stamp.toSec()-initialTime);
    fullLandMarks.channels[TrackingTimes].values.resize(fullLandMarks.points.size(),0);
}