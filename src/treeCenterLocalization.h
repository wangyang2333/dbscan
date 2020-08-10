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

#include <geometry_msgs/PointStamped.h>

#include <dlib/svm_threaded.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <tf/transform_listener.h>

#ifndef SRC_TREECENTERLOCALIZATION_H
#define SRC_TREECENTERLOCALIZATION_H

using namespace std;
class TreeCenterLocalization {
private:
    void tree_callback(const sensor_msgs::PointCloud::ConstPtr& landmarkPCL);
    ros::NodeHandle nh_;
    ros::Subscriber landmarkPCL_sub;

    ros::Publisher landmark_cloud_pub;
    ros::Publisher robot_pose_publisher;

    tf::TransformBroadcaster myTfBr;

    sensor_msgs::PointCloud map_cloud;

    string laser_name;
    string base_link_name;
    string odom_name;
    string map_name;

    bool firstTrackFlag;
public:
    TreeCenterLocalization(){
        ros::param::get("~laser_name",laser_name);
        ros::param::get("~base_link_name",base_link_name);
        ros::param::get("~odom_name",odom_name);
        ros::param::get("~map_name",map_name);
        landmarkPCL_sub = nh_.subscribe("/tree_center", 1, &TreeCenterLocalization::tree_callback, this);

        landmark_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 50);
        robot_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose_pr2", 10);

        sensor_msgs::PointCloud lanmark_cloud;
        map_cloud.header.frame_id = map_name;
        map_cloud.channels.resize(1);
        map_cloud.channels[0].name = "tree_id";
        map_cloud.points.clear();
        map_cloud.channels[0].values.clear();
        firstTrackFlag = true;
    }
};


#endif //SRC_TREECENTERLOCALIZATION_H
