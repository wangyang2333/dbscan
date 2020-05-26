//
// Created by xcy on 2020/5/18.
//

/*这里写了一个适用于高维和低维度的kmeans，其可以接受任意维度数据的聚类，
 * 但是其点的初始化（随机值或者随机点）不适用于高度集中的类。
 * 其中的随机点会被洒在同一个点上，导致类间融合，随机数也会导致个别点离其他点过近，
 * 特别要注意动态矩阵允许越界的情况。
 * */
#include "spectral_clustering.h"


double distance3(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) +
                (point1.y - point2.y)*(point1.y - point2.y) + (point1.z -point2.z)*(point1.z -point2.z));
}

Eigen::Vector3d P2V3(geometry_msgs::Point32 point){
    Eigen::Vector3d vector;
    vector[0] = point.x;
    vector[1] = point.y;
    vector[2] = point.z;
    return vector;
}

vector<double> P2stdV3(geometry_msgs::Point32 point){
    vector<double> vector;
    vector.resize(3);
    vector[0] = point.x;
    vector[1] = point.y;
    vector[2] = point.z;
    return vector;
}

geometry_msgs::Point32 V2P3(Eigen::Vector3d vector){
    geometry_msgs::Point32 point;
    point.x = vector[0];
    point.y = vector[1];
    point.z = vector[2];
    return point;
}
sensor_msgs::PointCloud KmeansforSpectralClustering(sensor_msgs::PointCloud PCL){
    //Initialization
    int K = 25;//number of means
    //int growthParam = 1;
    int Dimension = PCL.channels.size();
    PCL.channels.resize(PCL.channels.size()+1);
    PCL.channels.back().name = "cluster";
    PCL.channels.back().values.resize(PCL.points.size(),-1);
    vector<double> maxValue;
    vector<double> minValue;
    maxValue.resize(Dimension,-INFINITY);
    minValue.resize(Dimension,INFINITY);
    for(int i = 0; i < PCL.points.size(); i++){
        for(int n = 0; n <Dimension; n++){
            if(PCL.channels[n].values[i] < minValue[n]) minValue[n] = PCL.channels[n].values[i];
            if(PCL.channels[n].values[i] > maxValue[n]) maxValue[n] = PCL.channels[n].values[i];
        }
    }
    vector<Eigen::VectorXd> clusterCenters;
    clusterCenters.resize(K);
    srand((int)(time(NULL)));
    for(int k = 0; k < K; k++){
        clusterCenters[k].resize(Dimension);
        int randomIndex = floor(rand()/double(RAND_MAX)*PCL.points.size());
        for(int n = 0; n <Dimension; n++){
            //clusterCenters[k](n) = PCL.channels[n].values[randomIndex];
            clusterCenters[k](n) = rand()/double(RAND_MAX)*(maxValue[n] - minValue[n]) + minValue[n];
            ROS_INFO("num is:%f,  rand is: %f ",clusterCenters[k](n),rand()/double(RAND_MAX));
        }
        ROS_INFO("The center of cluster is %f,%f,%f ",clusterCenters[k](0),clusterCenters[k](1),clusterCenters[k](2));
    }
    double error = INFINITY;
    vector<Eigen::VectorXd> lastClusterCenters;
    int itCounter = 0;
    while(error > 1e-8 && itCounter <2500){
        itCounter++;
        //E-step compute r[n][k]
        for(int n = 0; n < PCL.points.size(); n++){
            double minDistance = INFINITY;
            double currentDistance;
            Eigen::VectorXd currentPts;
            currentPts.resize(Dimension);
            for(int j = 0; j <Dimension; j++){
                currentPts(j) = PCL.channels[j].values[n];
            }
            for(int k = 0; k < clusterCenters.size(); k++){
                currentDistance = (currentPts - clusterCenters[k]).norm();
                if(currentDistance < minDistance){
                    minDistance = currentDistance;
                    PCL.channels.back().values[n] = k;
                }
            }
        }
        lastClusterCenters = clusterCenters;

        //M-step compute u[k]
        for(int i = 0; i < clusterCenters.size(); i++){
            for(int j = 0; j <Dimension; j++){
                clusterCenters[i](j) = 0;
            }
        }

        double counter[K];
        for(int i=0; i < K; i++){
            counter[i] = 0;
        }
        for(int n = 0; n < PCL.points.size(); n++){
            counter[int(PCL.channels.back().values[n])]++;
            for(int j = 0; j <Dimension; j++){
                clusterCenters[int(PCL.channels.back().values[n])](j) += PCL.channels[j].values[n];
            }
        }
        for(int i = 0; i < clusterCenters.size(); i++){
            if(counter[i]==0){
//                int randomIndex = floor(rand()/double(RAND_MAX)*PCL.points.size());
//                for(int n = 0; n <Dimension; n++){
//                    //clusterCenters[i](n) = PCL.channels[n].values[randomIndex];
//                    clusterCenters[i](n) = rand()/double(RAND_MAX)*(maxValue[n] - minValue[n]) + minValue[n];
//                }
                continue;
            }
            for(int j = 0; j <Dimension; j++){
                clusterCenters[i](j) = clusterCenters[i](j)/counter[i];
            }
        }
        error = 0;
        for(int i = 0; i < clusterCenters.size(); i++){
            error = error + (clusterCenters[i] - lastClusterCenters[i]).norm();
        }

//        for(int i = 0; i < clusterCenters.size(); i++){
//            counter[0] = counter[0]*counter[i];
//        }
    }
    return PCL;
}

sensor_msgs::PointCloud spectralClustering(sensor_msgs::PointCloud PCL){
    /*Parameter Configuration*/
    int leafsize = 4;
    double min_extent = 0.0001;
    int knnSearchNum = 25;
    /*Octree Initialization*/
    ROS_INFO("There is all %d point(s).",int(PCL.points.size()));
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
    vector<int> point_indice;
    for(int i = 0; i < PCL.points.size(); i++){
        point_indice.push_back(i);
    }
    auto root = new OctreeNode(center, extent, point_indice, false);
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    buildOctree(root, PCL, center, extent, point_indice, leafsize, min_extent);
    endTime = clock();//计时结束
    cout << "The run Octree build time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    /*Construct Weight Matrix*/
    Eigen::MatrixXd W;
    W.resize(PCL.points.size(),PCL.points.size());
    W.setZero();
    for(int i = 0; i < PCL.points.size(); i++){
        //此处巨坑，每次使用外部变量前记得先清零,之前只用找同一个点
        resultIndex.clear();
        resultVector2.clear();
        worstDistance2.clear();
        searchOctreeNN(P2stdV3(PCL.points[i]), PCL, root, knnSearchNum);
        for(int k = 0; k <resultIndex.size(); k++){
            //W(i,resultIndex[k]) = distance3(PCL.points[i], PCL.points[resultIndex[k]]);
            if(i != resultIndex[k]){
                W(i,resultIndex[k]) = 1;
            }
        }
    }

    /*Construct Degree Matrix*/
    Eigen::MatrixXd D;
    D.resize(PCL.points.size(),PCL.points.size());
    D.setZero();
    Eigen::VectorXd rowSum = W.rowwise().sum();
    for(int i = 0; i < PCL.points.size(); i++){
        D(i,i) = rowSum(i);
    }

    /*Construct Laplacian Matrix*/
    Eigen::MatrixXd I, L;
    I.resize(PCL.points.size(),PCL.points.size());
    L.resize(PCL.points.size(),PCL.points.size());
    I.setIdentity();
    //L = D - W;
    L = I - D.inverse()*W;
    //Use SVD to avoid Complex eigenvalue?
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(L, Eigen::ComputeThinU | Eigen::ComputeThinV );

    //Eigen's eigen vecotors is normalized to 1
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd Singlr = svd.singularValues();

    /*open Channels*/
    int numOfClustering = 3;
    //cout<<V.block(0,V.cols()-3,V.rows(),3)<<endl;
    PCL.channels.resize(numOfClustering);
    string str;
    for(int i = 0; i < PCL.channels.size(); i++){
        PCL.channels[i].name = str.append(to_string(i));
        PCL.channels[i].values.resize(PCL.points.size());
    }
    for(int i = 0; i < PCL.points.size(); i++){
        for(int k = 0; k < PCL.channels.size(); k++){
            PCL.channels[k].values[i] = V(i,V.cols()-k-1);
        }
    }

    /*Kmeans Clustering*/
    ROS_INFO("kmeans");
    return KmeansforSpectralClustering(PCL);
}