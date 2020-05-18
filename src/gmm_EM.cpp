//
// Created by xcy on 2020/5/18.
//

#include "gmm_EM.h"

double normalDistribution(Eigen::Vector3d xn, Eigen::Vector3d uk, Eigen::Matrix3d SIGMA){
    double coeff = (1.0/pow(2.0*M_PI,3.0/2.0)) * (1.0/sqrt(SIGMA.determinant()));
    double power = -1.0/2.0 * (xn - uk).transpose()* SIGMA.inverse()* (xn - uk);
    return coeff * exp(power);
}

double distance2(geometry_msgs::Point32 point1, geometry_msgs::Point32 point2){
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) +
                (point1.y - point2.y)*(point1.y - point2.y) + (point1.z -point2.z)*(point1.z -point2.z));
}

Eigen::Vector3d P2V(geometry_msgs::Point32 point){
    Eigen::Vector3d vector;
    vector[0] = point.x;
    vector[1] = point.y;
    vector[2] = point.z;
    return vector;
}

geometry_msgs::Point32 V2P(Eigen::Vector3d vector){
    geometry_msgs::Point32 point;
    point.x = vector[0];
    point.y = vector[1];
    point.z = vector[2];
    return point;
}

sensor_msgs::PointCloud gmmEM(sensor_msgs::PointCloud PCL){
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

    vector<Eigen::Vector3d> clusterCenters;
    vector<Eigen::Matrix3d> clusterSIGMA;
    vector<double> clusterPI;

    clusterCenters.resize(K);
    clusterSIGMA.resize(K);
    clusterPI.resize(K);

    for(int i = 0; i < K; i++){
        geometry_msgs::Point32 tempPoint;
        clusterCenters[i][0] = rand()/double(RAND_MAX)*(xMax-xMin)+xMin;
        clusterCenters[i][1] = rand()/double(RAND_MAX)*(yMax-yMin)+yMin;
        clusterCenters[i][2] = rand()/double(RAND_MAX)*(zMax-zMin)+zMin;
//        clusterCenters[i][0] = (double(i)-1.0)*3.0;
//        clusterCenters[i][1] = 3;
//        clusterCenters[i][2] = 0;
        clusterSIGMA[i].setIdentity();
        clusterPI[i] = 1.0 / double(K);
    }

    double error = INFINITY;
    vector<Eigen::Vector3d> lastClusterCenters;
    Eigen::MatrixXd znk;
    znk.resize(PCL.points.size(), K);
    int counter= 100;
    while(counter!=0&&error>1e-3){
        counter--;
//        cout<<"center0:"<<clusterCenters[0]<<endl;
//        cout<<"center1:"<<clusterCenters[1]<<endl;
//        cout<<"center2:"<<clusterCenters[2]<<endl;
        //E-step compute znk[n][k]
        for(int n = 0; n < PCL.points.size(); n++){
            double sum;
            for(int k = 0; k < clusterCenters.size(); k++){
                znk(n, k) = clusterPI[k] * normalDistribution(P2V(PCL.points[n]), clusterCenters[k], clusterSIGMA[k]);
                sum += clusterPI[k] * normalDistribution(P2V(PCL.points[n]), clusterCenters[k], clusterSIGMA[k]);
            }
            for(int k = 0; k < clusterCenters.size(); k++){
                znk(n, k) = znk(n, k) / sum;
            }
        }
        lastClusterCenters = clusterCenters;
        //M-step compute u[k] SIGMA[k] PI[k]
        for(int i = 0; i < clusterCenters.size(); i++){
            clusterCenters[i].setZero();
            clusterSIGMA[i].setZero();
        }
        double N[K];
        for(int i=0; i < K; i++){
            N[i] = 0;
        }
        for(int n = 0; n < PCL.points.size(); n++){
            for(int k = 0; k < clusterCenters.size(); k++){
                clusterCenters[k] += znk(n, k) * P2V(PCL.points[n]);
                clusterSIGMA[k] += znk(n, k) * (P2V(PCL.points[n]) - clusterCenters[k])*(P2V(PCL.points[n]) - clusterCenters[k]).transpose();
                N[k] += znk(n,k);
            }
        }
        for(int i = 0; i < clusterCenters.size(); i++){
            if(N[i]==0){
                break;
            }
            clusterCenters[i] = clusterCenters[i] / N[i];
            clusterSIGMA[i] = clusterSIGMA[i] / N[i];
            clusterPI[i] = N[i] / PCL.points.size();
        }
        error = 0;
        for(int i = 0; i < clusterCenters.size(); i++){
            error = error + distance2(V2P(clusterCenters[i]),V2P(lastClusterCenters[i]));
        }
    }
    //OUTPUT
    for(int n = 0; n < PCL.points.size(); n++){
        int maxCluster;
        double maxZNK = 0;
        for(int k = 0; k < clusterCenters.size(); k++){
            if(znk(n, k) > maxZNK){
                maxZNK = znk(n, k);
                maxCluster = k;
            }
        }
        PCL.channels[0].values[n] = maxCluster;
    }
    //ROS_INFO("error: %f",error);
    for(int i = 0; i < clusterCenters.size(); i++){
        PCL.points.push_back(V2P(clusterCenters[i]));
        PCL.channels[0].values.push_back(i);
    }
    return PCL;
}