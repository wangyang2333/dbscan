//
// Created by xcy on 2020/10/13.
//

#include <ros/ros.h>
#include <iostream>
//#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>

using namespace std;

//void readCallback(const nav_msgs::OdometryConstPtr& odometry){
//    ROS_INFO("x = %f, z = %f", odometry->pose.pose.position.x, odometry->pose.pose.position.y);
//}
//
//
//int main(int argc, char** argv){
//    ros::init(argc, argv, "localization");
//    ros::NodeHandle nh_;
//    ros::Subscriber scan_sub = nh_.subscribe("/tf", 10000, readCallback);
//    ros::spin();
//    return 0;
//}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle node;
    tf::TransformListener listener;

    ros::Rate rate(100.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("x = %f, y = %f", transform.getOrigin().getX(), transform.getOrigin().getY());
        rate.sleep();
    }
    return 0;
}