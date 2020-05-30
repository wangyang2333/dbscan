//
// Created by xcy on 2020/5/4.
//

#ifndef SRC_OCTREE_NN_H
#define SRC_OCTREE_NN_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>

using namespace std;

//定义八叉树节点类
struct OctreeNode
{
    OctreeNode* children[8];
    vector<double> center;
    double extent;
    vector<int> point_indice;
    bool isLeaf;
    OctreeNode(vector<double> center_, double extent_, vector<int> point_indice_, bool isLeaf_){
        for(int i =0; i< 8; i++){
            children[i] = NULL;
        }
        center = center_;
        extent = extent_;
        point_indice = point_indice_;
        isLeaf = isLeaf_;
    }

};

class OctreeDriver{
private:
    double measureDistance(vector<double> point1, vector<double> point2, unsigned method);
    void addToWorstList(vector<double> toBeAdded, vector<double> goal, int index);
    void addToWorstListRadiusNN(vector<double> toBeAdded, vector<double> goal, int index);
    bool inside(vector<double> goal, OctreeNode *root);
    bool overlap(vector<double> goal, OctreeNode *root);

    vector<vector<double>> resultVector;
    vector<double> worstDistance;
    vector<int> resultIndex;
public:
    OctreeNode* buildOctree(OctreeNode* root, sensor_msgs::PointCloud& PCL, vector<double> center,
                            double extent, vector<int>& point_indice, int leafsize, double min_extent);
    bool searchOctreeRadiusNN(vector<double> goal, sensor_msgs::PointCloud& PCL, OctreeNode *root, double r);
    bool searchOctreeNN(vector<double> goal, sensor_msgs::PointCloud& PCL, OctreeNode *root, int k);
    void printOctree(OctreeNode *root, sensor_msgs::PointCloud& PCL);
    void octreeNNdemo(sensor_msgs::PointCloud& PCL);
    void octreeConstructFromPCL(sensor_msgs::PointCloud& PCL);
    vector<double> P2stdV(geometry_msgs::Point32 point){
        vector<double> vector;
        vector.resize(3);
        vector[0] = point.x;
        vector[1] = point.y;
        vector[2] = point.z;
        return vector;
    }
    vector<vector<double>> getResultVector(){return resultVector;}
    vector<double> getWorstDistance(){return worstDistance;}
    vector<int> getResultIndex(){return resultIndex;};
    void clearResult(){
        resultVector.clear();
        worstDistance.clear();
        resultIndex.clear();
    }

    OctreeNode* root;
};




#endif //SRC_OCTREE_NN_H
