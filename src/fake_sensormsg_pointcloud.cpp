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
    ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud>("chatter", 1000);
    ros::Rate loop_rate(10);
    sensor_msgs::PointCloud fakeCloud;
    fakeCloud.header.frame_id = "velodyne";

    geometry_msgs::Point32 tempPt;
    for(int i = 0; i<20; i++){
        tempPt.x = i*i;
        tempPt.y = 2+sin(i);
        tempPt.z = 3*i+4;
        fakeCloud.points.push_back(tempPt);
    }


    while (ros::ok())
    {

        chatter_pub.publish(fakeCloud);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}