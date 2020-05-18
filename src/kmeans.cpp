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
    int K = 3;
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
    while(error > 1e-1){
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
        for(int n = 0; n < PCL.points.size(); n++){
            clusterCenters[int(PCL.channels[0].values[n])].x += PCL.points[n].x;
            clusterCenters[int(PCL.channels[0].values[n])].y += PCL.points[n].y;
            clusterCenters[int(PCL.channels[0].values[n])].z += PCL.points[n].z;
        }
        for(int i = 0; i < clusterCenters.size(); i++){
            clusterCenters[i].x = clusterCenters[i].x/PCL.points.size();
            clusterCenters[i].y = clusterCenters[i].y/PCL.points.size();
            clusterCenters[i].z = clusterCenters[i].z/PCL.points.size();
        }
        error = 0;
        for(int i = 0; i < clusterCenters.size(); i++){
            error = error + distance(clusterCenters[i],lastClusterCenters[i]);
        }
    }
    return PCL;
}
