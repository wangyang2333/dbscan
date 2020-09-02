//
// Created by xcy on 2020/8/10.
//

#include <queue>
#include <ros/forwards.h>


#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>


#include <iostream>

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>



#include <geometry_msgs/PointStamped.h>

#include <dlib/svm_threaded.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <tf/transform_listener.h>

#include "octree_nn.h"

#ifndef SRC_TREECENTERLOCALIZATION_H
#define SRC_TREECENTERLOCALIZATION_H

using namespace std;



class TreeAtlas{
    friend class TreeCenterLocalization;
private:
    enum {TrackSuccess = 0, IdxInFullMap = 1, BirthTime = 2, TrackingTimes = 3, Activation = 4};
    double localMapRadius{};
    OctreeDriver oldDriver;
    sensor_msgs::PointCloud stableMap;
    sensor_msgs::PointCloud fullLandMarks;
    sensor_msgs::PointCloud localMap;
    string map_name, lidar_name;
    double stableTimeThreshould{}, stableTrackingThreshould{};
    double birthTimeThreshould{}, TrackingTimesThreshould{}, removalBeginTime{};
    double initialTime{};
    static void realTimeTransformPointCloud(const std::string & target_frame, const tf::Transform& net_transform,
                                     const ros::Time& target_time, const sensor_msgs::PointCloud & cloudIn,
                                     sensor_msgs::PointCloud & cloudOut) ;
    void mapRefine() const;
public:
    TreeAtlas(){
        ros::param::get("~stableTimeThreshould", stableTimeThreshould);
        ros::param::get("~stableTrackingThreshould",stableTrackingThreshould);
        ros::param::get("~birthTimeThreshould", birthTimeThreshould);
        ros::param::get("~TrackingTimesThreshould",TrackingTimesThreshould);
        ros::param::get("~removalBeginTime",removalBeginTime);
        localMap.channels.resize(4);
        localMap.channels[IdxInFullMap].name = "IdxInFullMap";
    }
    void atlasIntializationWithPCL(const sensor_msgs::PointCloud& initialPCL, string globalFrame);
    sensor_msgs::PointCloud getLocalMapWithTF(tf::StampedTransform currentTF);
    void addPointsToMapWithTF(sensor_msgs::PointCloud pointsToBeAdded, const tf::StampedTransform& currentTF);
    sensor_msgs::PointCloud getFullAtlas(){return fullLandMarks;}
    sensor_msgs::PointCloud getStableMap(){return stableMap;}
};



class TreeCenterLocalization {
    friend TreeAtlas;
private:
    enum {TrackSuccess = 0, IdxInFullMap = 1, BirthTime = 2, TrackingTimes = 3, Activation = 4};
    void tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL);
    ros::NodeHandle nh_;
    ros::Subscriber landmarkPCL_sub;

    ros::Publisher landmark_cloud_pub;

    tf::TransformBroadcaster my_br;
    tf::TransformListener listener;

//    tf::StampedTransform velodyne_to_base;
    tf::StampedTransform velodyne_to_map;


    TreeAtlas myAtlas;

    string lidar_name;
    string base_link_name;
    string odom_name;
    string map_name;
    double MaxCorrespondenceDistance, MaximumIterations, setTransformationEpsilon,EuclideanFitnessEpsilon;
    double localMapRadius;

    bool firstTrackFlag;
    Eigen::Matrix<float, 4, 4> lastPoseOfICP;
    Eigen::Matrix<float, 4, 4> initialGuessOfICP;

    geometry_msgs::PoseStamped my_pose;
    ros::Publisher my_pose_publisher;
    nav_msgs::Odometry my_odometry;
    ros::Publisher my_odometry_publisher;


    geometry_msgs::Point32 changeFrame(geometry_msgs::Point32 sourcePoint,
                                       string sourceFrame,
                                       const string& targetFrame);
    bool ICPwithStableMap(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL);
    bool ICPwithfullLandmarks(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL);
    static void posePredict(Eigen::Matrix<float, 4, 4> tfA,
                                                Eigen::Matrix<float, 4, 4> tfB,
                                                Eigen::Matrix<float, 4, 4>& result);

public:
    TreeCenterLocalization(){
        ros::param::get("~laser_name", lidar_name);
        ros::param::get("~base_link_name",base_link_name);
        ros::param::get("~odom_name",odom_name);
        ros::param::get("~map_name",map_name);


        ros::param::get("~MaxCorrespondenceDistance", MaxCorrespondenceDistance);
        ros::param::get("~MaximumIterations",MaximumIterations);
        ros::param::get("~setTransformationEpsilon",setTransformationEpsilon);
        ros::param::get("~EuclideanFitnessEpsilon",EuclideanFitnessEpsilon);
        ros::param::get("~localMapRadius",localMapRadius);

        myAtlas.localMapRadius = localMapRadius;
        landmarkPCL_sub = nh_.subscribe("/tree_center", 100, &TreeCenterLocalization::tree_callback, this);

        landmark_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("discrete_map", 100);
        my_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("my_pose", 100);
        my_odometry_publisher = nh_.advertise<nav_msgs::Odometry>("my_Odometry", 100);

        firstTrackFlag = true;
        lastPoseOfICP.setIdentity();
    }

};


#endif //SRC_TREECENTERLOCALIZATION_H
