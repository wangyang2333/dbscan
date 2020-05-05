//
// Created by xcy on 2020/5/3.
//

#include "kd_tree_nn.h"

vector<vector<double>> resultVector;
vector<double> worstDistance;

template<typename T>
vector<vector<T> > Transpose(vector<vector<T> > Matrix)
{
    unsigned row = Matrix.size();
    unsigned col = Matrix[0].size();
    vector<vector<T> > Trans(col,vector<T>(row,0));
    for (unsigned i = 0; i < col; ++i)
    {
        for (unsigned j = 0; j < row; ++j)
        {
            Trans[i][j] = Matrix[j][i];
        }
    }
    return Trans;
}

template <typename T>
T findMiddleValue(vector<T> vec)
{
    sort(vec.begin(),vec.end());
    auto pos = vec.size() / 2;
    return vec[pos];
}

void buildKdTree(KdTree* tree, vector<vector<double> >& data, unsigned depth)
{
    //样本的数量
    unsigned samplesNum = data.size();
    //终止条件
    if (samplesNum == 0)
    {
        return;
    }
    if (samplesNum == 1)
    {
        tree->root = data[0];
        return;
    }
    //样本的维度
    unsigned k = data[0].size();
    vector<vector<double> > transData = Transpose(data);
    //选择切分属性
    unsigned splitAttribute = depth % k;
    vector<double> splitAttributeValues = transData[splitAttribute];
    //选择切分值
    double splitValue = findMiddleValue(splitAttributeValues);
    //这里sort过，可以不用再逐个判断在左还是右边
    //cout << "splitValue" << splitValue  << endl;

    // 根据选定的切分属性和切分值，将数据集分为两个子集
    vector<vector<double> > subset1;
    vector<vector<double> > subset2;
    for (unsigned i = 0; i < samplesNum; ++i)
    {
        if (splitAttributeValues[i] == splitValue && tree->root.empty())
            tree->root = data[i];
        else
        {
            if (splitAttributeValues[i] < splitValue)
                subset1.push_back(data[i]);
            else
                subset2.push_back(data[i]);
        }
    }
    //子集递归调用buildKdTree函数



    if(!subset1.empty()){
        tree->leftChild = new KdTree;
        tree->leftChild->parent = tree;
        buildKdTree(tree->leftChild, subset1, depth + 1);
    }
    if(!subset2.empty()){
        tree->rightChild = new KdTree;
        tree->rightChild->parent = tree;
        buildKdTree(tree->rightChild, subset2, depth + 1);
    }
}

void printKdTree(KdTree *tree, unsigned depth)
{
    for (unsigned i = 0; i < depth; ++i)
        cout << "\t";

    for (vector<double>::size_type j = 0; j < tree->root.size(); ++j)
        cout << tree->root[j] << ",";
    cout << endl;
    if (tree->leftChild == NULL && tree->rightChild == NULL )//叶子节点
        return;
    else //非叶子节点
    {
        if (tree->leftChild != NULL)
        {
            for (unsigned i = 0; i < depth + 1; ++i)
                cout << "\t";
            cout << " left:";
            printKdTree(tree->leftChild, depth + 1);
        }

        cout << endl;
        if (tree->rightChild != NULL)
        {
            for (unsigned i = 0; i < depth + 1; ++i)
                cout << "\t";
            cout << "right:";
            printKdTree(tree->rightChild, depth + 1);
        }
        cout << endl;
    }
}

//计算空间中两个点的距离
double measureDistance(vector<double> point1, vector<double> point2, unsigned method)
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
//在kd树tree中搜索目标点goal的最近邻
//输入：目标点；已构造的kd树
//输出：目标点的最近邻


vector<vector<double>> addToWorstList(vector<double> toBeAdded, vector<double> goal, int k){
    if(resultVector.empty()){
        resultVector.resize(k);
        worstDistance.resize(k);
        for(int i = 0; i < worstDistance.size(); i++){//慎用auto！
            worstDistance[i] = 18800119782;//TODO:write deault vector.
        }
        for(int i = 0; i < resultVector.size(); i++){
            resultVector[i].push_back(18800119782);
            resultVector[i].push_back(18800119782);
            resultVector[i].push_back(18800119782);
        }
    }

    auto maxPosition = max_element(worstDistance.begin(), worstDistance.end());
    double currentDistance  = measureDistance(toBeAdded, goal, 0);
    if (measureDistance(toBeAdded, goal, 0) < *maxPosition ){
        resultVector[maxPosition - worstDistance.begin()] = toBeAdded;
        worstDistance[maxPosition - worstDistance.begin()] = currentDistance;
    }
    return resultVector;
}

void searchNearestNeighbor(vector<double> goal, KdTree *tree, int k, int depth)
{

    /*第一步：在kd树中找出包含目标点的叶子结点：从根结点出发，
    递归的向下访问kd树，若目标点的当前维的坐标小于切分点的
    坐标，则移动到左子结点，否则移动到右子结点，直到子结点为
    叶结点为止,以此叶子结点为“当前最近点”
    */
    KdTree *currentTree = tree;
    //int depth = 0;
    while(!currentTree->isLeaf())
    {
        unsigned dimensionIndex = depth % currentTree->root.size();//TODO:×2
        if(goal[dimensionIndex] <= currentTree->root[dimensionIndex]){
            if(currentTree->leftChild!=NULL)
            {
                currentTree = currentTree->leftChild;
            }else{
                currentTree = currentTree->rightChild;
            }
        }
        else{
            if(currentTree->rightChild!=NULL)
            {
                currentTree = currentTree->rightChild;
            }else{
                currentTree->leftChild;
            }
        }
        depth++;//TODO:考虑下终止条件！×2
    }
    /*第二步：递归地向上回退， 在每个结点进行如下操作：
    (a)如果该结点保存的实例比当前最近点距离目标点更近，则以该例点为“当前最近点”
    (b)当前最近点一定存在于某结点一个子结点对应的区域，检查该子结点的父结点的另
    一子结点对应区域是否有更近的点（即检查另一子结点对应的区域是否与以目标点为球
    心、以目标点与“当前最近点”间的距离为半径的球体相交）；如果相交，可能在另一
    个子结点对应的区域内存在距目标点更近的点，移动到另一个子结点，接着递归进行最
    近邻搜索；如果不相交，向上回退*/
    //The comparisons are performed using either operator< for the first version, or comp for the second;
    // An element is the largest if no other element does not compare less than it.
    // If more than one element fulfills this condition, the iterator returned points to
    // the first of such elements.

    //Check The Leaf
    addToWorstList(currentTree->root, goal, k);//TODO:check add one pt twice

    //Main Loop
    while(1){
        unsigned Idx = depth % currentTree->root.size();
        auto maxPosition = max_element(worstDistance.begin(), worstDistance.end());
        if (*maxPosition <= abs(currentTree->root[Idx] - currentTree->parent->root[Idx]) ){
            currentTree = currentTree->parent;
            depth--;
            if(currentTree->rightChild!=NULL){
                currentTree->rightChild->parent = NULL;
                currentTree->rightChild = NULL;
            }
            if(currentTree->leftChild!=NULL){
                currentTree->leftChild->parent = NULL;
                currentTree->leftChild = NULL;
            }
        }
        else{
            if(currentTree->anotherChild() != NULL){
                addToWorstList(currentTree->parent->root, goal, k);
                currentTree = currentTree->anotherChild();
                currentTree->killAnotherChild();
                //Go to Leaf
                while(!currentTree->isLeaf())
                {
                    unsigned dimensionIndex = depth % currentTree->root.size();
                    if(goal[dimensionIndex] <= currentTree->root[dimensionIndex]){
                        if(currentTree->leftChild!=NULL)
                        {
                            currentTree = currentTree->leftChild;
                        }else{
                            currentTree = currentTree->rightChild;
                        }
                    }
                    else{
                        if(currentTree->rightChild!=NULL)
                        {
                            currentTree = currentTree->rightChild;
                        }else{
                            currentTree->leftChild;
                        }
                    }
                    depth++;
                }
                searchNearestNeighbor(goal, currentTree, k, depth);
            }
            else{
                currentTree = currentTree->parent;
                depth--;
                if(currentTree->rightChild!=NULL){
                    currentTree->rightChild->parent = NULL;
                    currentTree->rightChild = NULL;
                }
                if(currentTree->leftChild!=NULL){
                    currentTree->leftChild->parent = NULL;
                    currentTree->leftChild = NULL;
                }
            }
        }
        if(currentTree->parent==NULL && currentTree->parent==NULL && currentTree->parent==NULL){
            return;
        }
    }
}

void KD_TREE_NN(sensor_msgs::PointCloud& PCL){
    //transform data size
    vector<vector<double>> PCL_data;
    //ROS_INFO("There is all %d point(s).",PCL.points.size());
    PCL_data.resize(PCL.points.size());
    for(int i = 0; i <PCL.points.size(); i++){
        PCL_data[i].push_back(PCL.points[i].x);
        PCL_data[i].push_back(PCL.points[i].y);
        PCL_data[i].push_back(PCL.points[i].z);
    }

    //test KD_tree build
//    PCL_data.resize(13);
//    for(int i = 0; i <13; i++){
//        PCL_data[i].push_back(i+1);
//        PCL_data[i].push_back(14-2*i);
//        PCL_data[i].push_back(i*i-4*i);
//    }

    //build KDTree
    KdTree* kdTree = new KdTree;
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    buildKdTree(kdTree, PCL_data, 0);;
    endTime = clock();//计时结束
    cout << "The run KDtree build time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    //printKdTree(kdTree, 0);
    vector<double> goal;
    goal.push_back(8.1);
    goal.push_back(12);
    goal.push_back(-3);
    startTime = clock();//计时开始
    searchNearestNeighbor(goal, kdTree, 3, 0);
    endTime = clock();//计时结束
    cout << "The run KDtree search time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    for(int i = 0; i <3; i ++ ){
        cout<<"x="<<resultVector[i][0]<<",y="<<resultVector[i][1]<<",z="<<resultVector[i][2]<<endl;
    }
}

void testPCL(sensor_msgs::PointCloud& PCL){
    srand ((unsigned int) time (NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 PCL2;
    sensor_msgs::convertPointCloudToPointCloud2(PCL, PCL2);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(PCL2,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);



    float resolution = 0.0001;

    clock_t startTime, endTime;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (cloud);
    startTime = clock();//计时开始
    octree.addPointsFromInputCloud ();
    endTime = clock();//计时结束
    cout << "The PCL Octree build run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

    // K nearest neighbor search
    int K = 4;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;
    startTime = clock();//计时开始
    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }
    endTime = clock();//计时结束
    cout << "The PCL Octree seach run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

}