//
// Created by xcy on 2020/5/18.
//

#include "kmeans.h"

double distance(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) +
    (point1.y - point2.y)*(point1.y - point2.y) + (point1.z -point2.z)*(point1.z -point2.z));
}

sensor_msgs::PointCloud Kmeans(sensor_msgs::PointCloud PCL){
    //Initialization
    int K = 3;
    PCL.channels.clear();
    PCL.channels.resize(1);
    PCL.channels[0].name = "cluster";
    PCL.channels[0].values.resize(PCL.points.size());
    double xMax, xMin, yMax, yMin, zMax, zMin;
    xMax = yMax = zMax = -INFINITY;
    xMin = yMin = zMin = INFINITY;
    for(int i = 0; i < PCL.points.size(); i++){
        if(PCL.points[i].x < xMin) xMin = PCL.points[i].x;
        if(PCL.points[i].y < yMin) yMin = PCL.points[i].y;
        if(PCL.points[i].z < zMin) zMin = PCL.points[i].z;
        if(PCL.points[i].x > xMax) xMax = PCL.points[i].x;
        if(PCL.points[i].y > yMax) yMax = PCL.points[i].y;
        if(PCL.points[i].z > zMax) zMax = PCL.points[i].z;
    }
    srand((int)(time(NULL)));
    vector<geometry_msgs::Point32> clusterCenters;
    for(int i = 0; i < K; i++){
        geometry_msgs::Point32 tempPoint;
        //cout<<rand()/double(RAND_MAX)<<endl;
        tempPoint.x = rand()/double(RAND_MAX)*(xMax-xMin)+xMin;
        tempPoint.y = rand()/double(RAND_MAX)*(yMax-yMin)+yMin;
        tempPoint.z = rand()/double(RAND_MAX)*(zMax-zMin)+zMin;
        clusterCenters.push_back(tempPoint);
    }

    double error = INFINITY;
    vector<geometry_msgs::Point32> lastClusterCenters;
    while(error > 1e-6){
//        ROS_INFO("The center of cluster is %f,%f,%f ",clusterCenters[0].x,clusterCenters[0].y,clusterCenters[0].z);
//        ROS_INFO("The center of cluster is %f,%f,%f ",clusterCenters[1].x,clusterCenters[1].y,clusterCenters[1].z);
//        ROS_INFO("The center of cluster is %f,%f,%f ",clusterCenters[2].x,clusterCenters[2].y,clusterCenters[2].z);
        //E-step compute r[n][k]
        for(int n = 0; n < PCL.points.size(); n++){
            double minDistance = INFINITY;
            double currentDistance;
            for(int k = 0; k < clusterCenters.size(); k++){
                currentDistance = distance(PCL.points[n],clusterCenters[k]);
                if(currentDistance < minDistance){
                    minDistance = currentDistance;
                    PCL.channels[0].values[n] = k;
                }
            }
        }
        lastClusterCenters = clusterCenters;
        //M-step compute u[k]
        for(int i = 0; i < clusterCenters.size(); i++){
            clusterCenters[i].x = 0;
            clusterCenters[i].y = 0;
            clusterCenters[i].z = 0;
        }
        double counter[K];
        for(int i=0; i < K; i++){
            counter[i] = 0;
        }
        for(int n = 0; n < PCL.points.size(); n++){
            counter[int(PCL.channels[0].values[n])]++;
            clusterCenters[int(PCL.channels[0].values[n])].x += PCL.points[n].x;
            clusterCenters[int(PCL.channels[0].values[n])].y += PCL.points[n].y;
            clusterCenters[int(PCL.channels[0].values[n])].z += PCL.points[n].z;
        }
        for(int i = 0; i < clusterCenters.size(); i++){
            if(counter[i]==0){
                break;
            }
            clusterCenters[i].x = clusterCenters[i].x/counter[i];
            clusterCenters[i].y = clusterCenters[i].y/counter[i];
            clusterCenters[i].z = clusterCenters[i].z/counter[i];
        }
        error = 0;
        for(int i = 0; i < clusterCenters.size(); i++){
            error = error + distance(clusterCenters[i],lastClusterCenters[i]);
        }
    }
    //ROS_INFO("error: %f",error);
    for(int i = 0; i < clusterCenters.size(); i++){
        PCL.points.push_back(clusterCenters[i]);
        PCL.channels[0].values.push_back(i);
    }
    return PCL;
}
