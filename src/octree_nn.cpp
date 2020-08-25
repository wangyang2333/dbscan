//
// Created by xcy on 2020/5/4.
//

#include "octree_nn.h"

//创建八叉树
OctreeNode* OctreeDriver::buildOctree(OctreeNode* root, sensor_msgs::PointCloud& PCL, vector<double> center,
                                      double extent, vector<int>& point_indice, int leafsize, double min_extent){
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
        }
        for(int i = 0; i < 8; i++){
            vector<double> childCenter;
            childCenter.clear();
            childCenter.push_back(center[0] + (((i&1)!=0)?0.5:-0.5)*extent);
            childCenter.push_back(center[1] + (((i&2)!=0)?0.5:-0.5)*extent);
            childCenter.push_back(center[2] + (((i&4)!=0)?0.5:-0.5)*extent);
            double childExtent = 0.5 * extent;
            root->children[i] = buildOctree(root->children[i], PCL,
                                            childCenter, childExtent, childrenPointIndice[i], leafsize, min_extent);
        }
    }
    return root;
}

void OctreeDriver::printOctree(OctreeNode *root, sensor_msgs::PointCloud& PCL)
{
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

double OctreeDriver::measureDistance(vector<double> point1, vector<double> point2, unsigned method)
{
    if (point1.size() != point2.size())
    {
        cerr << "Dimensions don't match！！"<<" PT1.size: "<<point1.size()<<", PT2.size: "<<point2.size();
        exit(1);
    }
    switch (method)
    {
        case 0://欧氏距离
        {
            double res = 0;
            for (vector<double>::size_type i = 0; i < point1.size(); ++i)
            {
                res += pow((point1[i] - point2[i]), 2);
            }
            return sqrt(res);
        }
        case 1://曼哈顿距离
        {
            double res = 0;
            for (vector<double>::size_type i = 0; i < point1.size(); ++i)
            {
                res += abs(point1[i] - point2[i]);
            }
            return res;
        }
        default:
        {
            cerr << "Invalid method!!" << endl;
            return -1;
        }
    }
}

void OctreeDriver::addToWorstList(vector<double> toBeAdded, vector<double> goal, int index){
    auto maxPosition = max_element(worstDistance.begin(), worstDistance.end());
    double currentDistance  = measureDistance(toBeAdded, goal, 0);
    if (measureDistance(toBeAdded, goal, 0) < *maxPosition ){
        resultVector[maxPosition - worstDistance.begin()] = toBeAdded;
        worstDistance[maxPosition - worstDistance.begin()] = currentDistance;
        resultIndex[maxPosition - worstDistance.begin()] = index;
    }
    return;
}

void OctreeDriver::addToWorstListRadiusNN(vector<double> toBeAdded, vector<double> goal, int index){
    auto maxPosition = max_element(worstDistance.begin(), worstDistance.end());
    double currentDistance  = measureDistance(toBeAdded, goal, 0);
    if (measureDistance(toBeAdded, goal, 0) < *maxPosition ){
        resultVector.push_back(toBeAdded);
        resultIndex.push_back(index);
    }
    return;
}


bool OctreeDriver::inside(vector<double> goal, OctreeNode *root){
    double offset;
    auto maxPosition = max_element(worstDistance.begin(), worstDistance.end());
    for(int i = 0; i < 3; i++){
        offset = abs(root->center[i] - goal[i]) + *maxPosition;
        if(offset > root->extent)return false;
    }
    return true;
}

bool OctreeDriver::overlap(vector<double> goal, OctreeNode *root){
    vector<double> query_offset;

    auto maxPosition = max_element(worstDistance.begin(), worstDistance.end());
    for(int i = 0; i < 3; i++){
        query_offset.push_back(abs(root->center[i] - goal[i]));
    }
    //Case Uno
    double max_dist = *maxPosition + root->extent;
    for(int i = 0; i < 3; i++){
        if(query_offset[i] > max_dist)return false;
    }
    //Case Dos
    int overlap_counter = 0;
    for(int i = 0; i < 3; i++){
        if(query_offset[i] < root->extent)overlap_counter++;
    }
    if(overlap_counter>=2)return true;
    //Case Tres
    double x_diff = max(query_offset[0],0.0);
    double y_diff = max(query_offset[1],0.0);
    double z_diff = max(query_offset[2],0.0);
    return (x_diff * x_diff + y_diff * y_diff + z_diff * z_diff < *maxPosition * *maxPosition);
}

bool OctreeDriver::searchOctreeNN(vector<double> goal, sensor_msgs::PointCloud& PCL, OctreeNode *root, int k){
    if(resultVector.empty()){
        resultVector.resize(k);
        worstDistance.resize(k);
        resultIndex.resize(k);
        for(int i = 0; i < worstDistance.size(); i++){//慎用auto！
            worstDistance[i] = INFINITY;//TODO:write deault vector.
        }
        for(int i = 0; i < resultVector.size(); i++){
            resultVector[i].push_back(0);
            resultVector[i].push_back(0);
            resultVector[i].push_back(0);
        }
        for(int i = 0; i < resultIndex.size(); i++){
            resultIndex[i]=-1;
        }
    }
    if(root==NULL)return false;
    if(root->isLeaf && root->point_indice.size() > 0){
        for(int i = 0; i < root->point_indice.size(); i++){
            vector<double> toBeAdded;
            toBeAdded.push_back(PCL.points[root->point_indice[i]].x);
            toBeAdded.push_back(PCL.points[root->point_indice[i]].y);
            toBeAdded.push_back(PCL.points[root->point_indice[i]].z);
            addToWorstList(toBeAdded, goal, root->point_indice[i]);
        }
        return inside(goal, root);
    }
    int morton_code = 0;
    if(goal[0] > root->center[0])morton_code = morton_code|1;
    if(goal[1] > root->center[1])morton_code = morton_code|2;
    if(goal[2] > root->center[2])morton_code = morton_code|4;
    if(searchOctreeNN(goal, PCL, root->children[morton_code], k))return true;
    for(int i = 0; i < 8; i++){
        if(i == morton_code || root->children[i] == NULL)continue;
        if(overlap(goal,root->children[i])== false)continue;
        if(searchOctreeNN(goal, PCL, root->children[i], k))return true;
    }
    return inside(goal, root);
}

bool OctreeDriver::searchOctreeRadiusNN(vector<double> goal, sensor_msgs::PointCloud& PCL, OctreeNode *root, double r){
    if(resultVector.empty()){
        worstDistance.resize(1, r);
    }
    if(root==NULL)return false;
    if(root->isLeaf && root->point_indice.size() > 0){
        for(int i = 0; i < root->point_indice.size(); i++){
            vector<double> toBeAdded;
            toBeAdded.push_back(PCL.points[root->point_indice[i]].x);
            toBeAdded.push_back(PCL.points[root->point_indice[i]].y);
            toBeAdded.push_back(PCL.points[root->point_indice[i]].z);
            addToWorstListRadiusNN(toBeAdded, goal, root->point_indice[i]);
        }
        return inside(goal, root);
    }
    int morton_code = 0;
    if(goal[0] > root->center[0])morton_code = morton_code|1;
    if(goal[1] > root->center[1])morton_code = morton_code|2;
    if(goal[2] > root->center[2])morton_code = morton_code|4;
    if(searchOctreeRadiusNN(goal, PCL, root->children[morton_code], r))return true;
    for(int i = 0; i < 8; i++){
        if(i == morton_code || root->children[i] == NULL)continue;
        if(overlap(goal,root->children[i])== false)continue;
        if(searchOctreeRadiusNN(goal, PCL, root->children[i], r))return true;
    }
    return inside(goal, root);
}

void OctreeDriver::octreeConstructFromPCL(sensor_msgs::PointCloud &PCL) {
    //ROS_INFO("There is all %d point(s).",int(PCL.points.size()));
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
    root = new OctreeNode(center, extent, point_indice, false);
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    buildOctree(root, PCL, center, extent, point_indice, leafsize, min_extent);
    endTime = clock();//计时结束
    //cout << "The run Octree build time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}

void OctreeDriver::octreeNNdemo(sensor_msgs::PointCloud& PCL){
    octreeConstructFromPCL(PCL);
    vector<double> goal;
    goal.push_back(11.8);
    goal.push_back(27.0);
    goal.push_back(-3.3);
    //printOctree(root, PCL);
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    //searchOctreeNN(goal, PCL, root, 3);
    searchOctreeRadiusNN(goal, PCL, root, 4.0);
    endTime = clock();//计时结束
    cout << "The run Octree search time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    for(int i = 0; i < resultVector.size(); i++){
        cout<<"x="<<resultVector[i][0]<<",y="<<resultVector[i][1]<<",z="<<resultVector[i][2]<<endl;
        cout<<resultIndex[i]<<endl;
    }
}



