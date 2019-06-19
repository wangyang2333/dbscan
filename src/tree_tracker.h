//
// Created by xcy on 19-6-11.
//
#include <queue>
#include <sensor_msgs/Imu.h>
#include <ros/forwards.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <mutex>
#include <condition_variable>
#include <thread>
#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//------------------------------------
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>

#include <dlib/svm_threaded.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <tf/transform_listener.h>
#ifndef SRC_TREE_TRACKER_H
#define SRC_TREE_TRACKER_H

using namespace cv;
using namespace std;
using namespace dlib;

class tree{
public:
    double sita;
    double rou;
    double tree_id;
    tree(double x_,double y_, double id_){
        sita = x_;
        rou = y_;
        tree_id = id_;
    }
};

class tree_tracker{
private:
    void tree_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    ros::NodeHandle nh_;
    std::mutex tree_mtx;
    bool first_track_flag;
    std::vector<tree> this_trees;


    sensor_msgs::LaserScan tree_followed;
    sensor_msgs::PointCloud map_cloud;
    geometry_msgs::PoseStamped pr2_pose;

    ros::Publisher follow_pub;
    ros::Publisher landmark_cloud_pub;
    ros::Publisher pr2_pose_publisher;
    ros::Subscriber scan_sub;
    tf::TransformListener listener;

    double norm_distance(tree ta, tree tb);
    void track_tree();
    int find_bad_tree(std::vector<tree> tree_more , std::vector<tree> tree_less);
    void add_to_map(tree new_landmark);
    void remove_from_map(tree landmark_to_remove);
    void change_frame(geometry_msgs::Point32 pt_in, string frame_in, geometry_msgs::Point32& pt_out, string frame_out);
    void change_frame(tree tree_in, string frame_in, geometry_msgs::Point32& pt_out, string frame_out);
    void localize();
    double last_sita = 0.0;
    tf::Transform my_transform;
    tf::TransformBroadcaster my_br;

public:
    static double pts32_error(geometry_msgs::Point32 pts1, geometry_msgs::Point32 pts2);
    std::vector<tree> this_track;
    tree_tracker(){
        first_track_flag = true;
        scan_sub = nh_.subscribe("/tree_pt", 1, &tree_tracker::tree_callback, this);
        follow_pub = nh_.advertise<sensor_msgs::LaserScan>("/tree_followed",1);
        landmark_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 50);
        pr2_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose_pr2",50);
        sensor_msgs::PointCloud lanmark_cloud;
        map_cloud.header.frame_id = "odom_combined";
        map_cloud.channels.resize(1);
        map_cloud.channels[0].name = "tree_id";
        map_cloud.points.clear();
        map_cloud.channels[0].values.clear();

        my_transform.setOrigin(tf::Vector3(0,0,0));
        my_transform.setRotation(tf::createQuaternionFromYaw(0));
        my_br.sendTransform(tf::StampedTransform(my_transform,ros::Time::now(),"odom_combined","case_link"));
    }
};






#endif //SRC_TREE_TRACKER_H
