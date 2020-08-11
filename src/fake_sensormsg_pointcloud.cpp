//
// Created by xcy on 2020/8/11.
//

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "fake_cloud_publisher");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud>("/tree_center", 1000);
    ros::Rate loop_rate(1);
    sensor_msgs::PointCloud fakeCloud;
    fakeCloud.header.frame_id = "velodyne";

    geometry_msgs::Point32 tempPt;
    for(int i = 0; i<5; i++){
        tempPt.x = i*i/3;
        tempPt.y = 2+sin(i);
        tempPt.z = 3*i+4/10;
        fakeCloud.points.push_back(tempPt);
    }

    int count = 0;
    while (ros::ok())
    {
        chatter_pub.publish(fakeCloud);
        if(count>5){
            geometry_msgs::Point32 tempPts;
            tempPts = fakeCloud.points[4];
            tempPts.x = 100;
            fakeCloud.points[0]=tempPt;
        }
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}