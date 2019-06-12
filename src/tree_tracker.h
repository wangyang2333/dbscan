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
    sensor_msgs::LaserScan tree_followed;
    ros::Publisher follow_pub;
    ros::Subscriber scan_sub;
    bool first_track_flag;
    std::vector<tree> this_track;
    std::vector<tree> this_trees;
    double norm_distance(tree ta, tree tb);
    void track_tree();
    int find_bad_tree(std::vector<tree> tree_more , std::vector<tree> tree_less);
public:
    tree_tracker(){
        first_track_flag = true;
        scan_sub = nh_.subscribe("/tree_pt", 1, &tree_tracker::tree_callback, this);
        follow_pub = nh_.advertise<sensor_msgs::LaserScan>("/tree_followed",1);
    }
};






#endif //SRC_TREE_TRACKER_H
