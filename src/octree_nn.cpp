//
// Created by xcy on 2020/5/4.
//

#include "octree_nn.h"

//创建八叉树
OctreeNode* buildOctree(OctreeNode* root, sensor_msgs::PointCloud& PCL, vector<double> center,
        double extent, vector<int> point_indice, int leafsize, double min_extent){
    if(point_indice.size()==0) return NULL;
    if(root==NULL) root = new OctreeNode(center, extent, point_indice, false);
    if(point_indice.size()<= leafsize || extent <= min_extent) root->isLeaf = true;
    else{
        root->isLeaf = false;
        vector<int> childrenPointIndice[8];
        for(int i = 0; i < point_indice.size(); i++){
            int mortoncode = 0;
            if( PCL.points[point_indice[i]].x > center[0])mortoncode = mortoncode|1;
            if( PCL.points[point_indice[i]].y > center[1])mortoncode = mortoncode|2;
            if( PCL.points[point_indice[i]].z > center[2])mortoncode = mortoncode|4;
            childrenPointIndice[mortoncode].push_back(point_indice[i]);
            //ROS_INFO("mortoncode:%d",mortoncode);
        }
        for(int i = 0; i < 8; i++){
            vector<double> childCenter;
            childCenter.clear();
            childCenter.push_back(center[0] + (((i&1)!=0)?0.5:-0.5)*extent);
            childCenter.push_back(center[1] + (((i&2)!=0)?0.5:-0.5)*extent);
            childCenter.push_back(center[2] + (((i&4)!=0)?0.5:-0.5)*extent);
            double childExtent = 0.5 * extent;
            //ROS_INFO("extent = %f",childExtent);
            //ROS_INFO("i=%d, center:%f,%f,%f",i ,childCenter[0],childCenter[1],childCenter[2]);
            root->children[i] = buildOctree(root->children[i], PCL,
                                            childCenter, childExtent, childrenPointIndice[i], leafsize, min_extent);
        }
    }
    return root;
}

void printOctree(OctreeNode *root, sensor_msgs::PointCloud PCL)
{
    ROS_INFO("extent:%f",root->extent);
    if(root->isLeaf){
        for(int i = 0; i < root->point_indice.size(); i++){
            ROS_INFO(" PTS_indice:%d",root->point_indice[i]);
            ROS_INFO("   %f,%f,%f",PCL.points[root->point_indice[i]].x, PCL.points[root->point_indice[i]].y, PCL.points[root->point_indice[i]].z);
        }
        return;
    }else{
        for(int i = 0; i < 8; i++){
            if(root->children[i]==NULL) continue;
            else printOctree(root->children[i], PCL);
        }
        return;
    }
}


void OCTREE_NN(sensor_msgs::PointCloud& PCL){
    //ROS_INFO("There is all %d point(s).",PCL.points.size());
    double max_x=-INFINITY, max_y=-INFINITY, max_z=-INFINITY, min_x =INFINITY, min_y=INFINITY, min_z=INFINITY;
    for(int i = 0; i < PCL.points.size();  i++ ){
        if(PCL.points[i].x < min_x) min_x = PCL.points[i].x;
        if(PCL.points[i].x > max_x) max_x = PCL.points[i].x;
        if(PCL.points[i].y < min_y) min_y = PCL.points[i].y;
        if(PCL.points[i].y > max_y) max_y = PCL.points[i].y;
        if(PCL.points[i].z < min_z) min_z = PCL.points[i].z;
        if(PCL.points[i].z > max_z) max_z = PCL.points[i].z;
    }
    double extent = max( max(max_x-min_x, max_y-min_y), max_z-min_z )/2.0;
    vector<double> center;
    center.push_back((max_x + min_x)/2.0);
    center.push_back((max_y + min_y)/2.0);
    center.push_back((max_z + min_z)/2.0);
//    ROS_INFO("extent = %f",extent);
//    ROS_INFO("center:%f,%f,%f",center[0],center[1],center[2]);
//    ROS_INFO("max:%f,%f,%f",max_x,max_y,max_z);
//    ROS_INFO("min:%f,%f,%f",min_x,min_y,min_z);
    vector<int> point_indice;
    for(int i = 0; i < PCL.points.size(); i++){
        point_indice.push_back(i);
    }
    int leafsize = 4;
    double min_extent = 0.0001;
    auto root = new OctreeNode(center, extent, point_indice, false);
    buildOctree(root, PCL, center, extent, point_indice, leafsize, min_extent);
    //printOctree(root, PCL);
}

