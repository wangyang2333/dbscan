//
// Created by xcy on 2020/5/3.
//

#ifndef SRC_KD_TREE_NN_H
#define SRC_KD_TREE_NN_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include "kd_tree_nn.h"

using namespace std;

struct KdTree{
    vector<double> root;
    KdTree* parent;
    KdTree* leftChild;
    KdTree* rightChild;
    //默认构造函数
    KdTree(){parent = leftChild = rightChild = NULL;}
    //判断kd树是否为空
    bool isEmpty()
    {
        return root.empty();
    }
    //判断kd树是否只是一个叶子结点
    bool isLeaf()
    {
        return (!root.empty()) &&
               rightChild == NULL && leftChild == NULL;
    }
    //判断是否是树的根结点
    bool isRoot()
    {
        return (!isEmpty()) && parent == NULL;
    }
    //判断该子kd树的根结点是否是其父kd树的左结点
    bool isLeft()
    {
        return parent->leftChild->root == root;
    }
    //判断该子kd树的根结点是否是其父kd树的右结点
    bool isRight()
    {
        return parent->rightChild->root == root;
    }

    KdTree* anotherChild()
    {
        if(parent->leftChild == NULL){
            return NULL;
        }
        if( isLeft())return parent->rightChild;
        else return parent->leftChild;
    }

    void killAnotherChild()
    {
        if(parent->leftChild == NULL){
            return;
        }
        if( isLeft()){
            parent->rightChild->parent = NULL;
            parent->rightChild = NULL;
        }
        else{
            parent->leftChild->parent = NULL;
            parent->leftChild = NULL;
        }
    }
};

template<typename T>
vector<vector<T> > Transpose(vector<vector<T> > Matrix);

template <typename T>
T findMiddleValue(vector<T> vec);

void buildKdTree(KdTree* tree, vector<vector<double> > data, unsigned depth);

void printKdTree(KdTree *tree, unsigned depth);

//计算空间中两个点的距离
double measureDistance(vector<double> point1, vector<double> point2, unsigned method);

vector<vector<double>> addToWorstList(vector<double> toBeAdded, vector<double> goal, int k);

void searchNearestNeighbor(vector<double> goal, KdTree *tree, int k, int depth);

void KD_TREE_NN(sensor_msgs::PointCloud& PCL);

#endif //SRC_KD_TREE_NN_H
