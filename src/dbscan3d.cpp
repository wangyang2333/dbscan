/*
 * DBSCAN For VLP-16.
 * All right reserved.
 */

#include <queue>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <mutex>

#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

//#include "kd_tree_nn.h"
//#include "octree_nn.h"
//#include "kmeans.h"
//#include "gmm_EM.h"
//#include "spectral_clustering.h"
#include "groundRemovalRANSAC.h"
#include "dbscan_clustering.h"

using namespace cv;
using namespace std;

std::mutex scan_lock;
double scan_num360;
double scan_range;
int min_cluster;
double distance_max;
double distance_min;
double height_max;
double height_min;
int MinPts;
double EPS, tree_residual, tree_radius_max, tree_radius_min;

double groundZMax;
double groundZMin;
double inlierRatio;
int sampleNum;
double confidence;
double inlinerThreshold;
double ratioCondition;
double upperBorder;


class point{
public:
    float x;
    float y;
    float z;
    float rou;
    float sita;
    int cluster=0;
    int pointType=1;//1 noise 2 border 3 core
    int pts=0;//points in MinPts
    vector<int> corepts;
    int visited = 0;
    point (){}
    point (float a,float b,float c ,int d){
        x = a;
        y = b;
        z = c;
        cluster = d;
        rou = sqrt(a*a + b*b);
        sita = atan2(b , a);
    }
};

float squareDistance(point a,point b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    //return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)/5.0);
}

struct circleResidual {
    circleResidual(double xi, double yi)
            : xi_(xi),yi_(yi) {}
    template <typename T> bool operator()(const T* const x,
                                          const T* const y,
                                          const T* const r,
                                          T* residual) const {
        residual[0] = r[0]*r[0] - (x[0]-xi_)*(x[0]-xi_) - (y[0]-yi_)*(y[0]-yi_);//TODO
        return true;
    }
private:
    const double xi_;
    const double yi_;
};
ros::Publisher cloud_pub;
ros::Publisher tree_cloud_pub;
ros::Publisher circle_pub;
ros::Publisher tree_visual_cloud_pub;
sensor_msgs::LaserScan centers;

void DBSCAN(sensor_msgs::PointCloud& dataset,double eps,int minpts){//按照xy密度来进行聚类。
    /*Project 3D to 2D*/
    sensor_msgs::PointCloud tempz = dataset;
    sensor_msgs::PointCloud tempTrue = dataset;
    for(int i = 0; i < tempTrue.points.size(); i++){
        //自己写的八叉树难以处理某一维度全为0的情况
        tempTrue.points[i].z = 0.05*rand() / double(RAND_MAX);
    }

    /*Run DBscan*/
    clock_t startTime,endTime;
    startTime = clock();//计时开始
    DBscanDriver oldDriver;
    oldDriver.setEPSandMinPts(eps, minpts);
    oldDriver.dbscanClustering(tempTrue);
    dataset.channels = oldDriver.PCLforOutput.channels;




    endTime = clock();//计时结束
    cout << "The DBSCAN Clustering run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    /*Remove little cluster and add Residual*/
    auto ClusterBegin = dataset.channels[DBscanDriver::cluster].values.begin();
    auto ClusterEnd = dataset.channels[DBscanDriver::cluster].values.end();
    int maxCluster = (int)*max_element(ClusterBegin, ClusterEnd);

    /*Get cluster index*/
    vector<vector<int>> currentClusterIdx;
    currentClusterIdx.resize(maxCluster+1);

    for(int j = 0; j < dataset.points.size(); j++){
        currentClusterIdx[(int)dataset.channels[DBscanDriver::cluster].values[j]].push_back(j);
    }
    /*Remove little cluster*/
    for(int j = 1; j < currentClusterIdx.size(); j++){
        if(currentClusterIdx[j].size() < min_cluster){
            for(int i = 0; i < currentClusterIdx[j].size(); i++){
                dataset.channels[DBscanDriver::type].values[currentClusterIdx[j][i]] = DBscanDriver::little;
                dataset.channels[DBscanDriver::cluster].values[currentClusterIdx[j][i]] = 0.0;
            }
            currentClusterIdx.erase(currentClusterIdx.begin() + j);
            j--;
        }
    }

    for(int j = 1; j < currentClusterIdx.size(); j++) {
        //优化初始值把类中第一个点的位置复制给他
        double x = dataset.points[currentClusterIdx[j].front()].x;
        double y = dataset.points[currentClusterIdx[j].front()].y;
        double r = 0.15;

        ceres::Problem problem;
        for (int i = 0; i < currentClusterIdx[j].size() ; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<circleResidual, 1, 1, 1, 1>(
                            new circleResidual(dataset.points[currentClusterIdx[j][i]].x, dataset.points[currentClusterIdx[j][i]].y)),
                    //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &x, &y, &r);
        }
        //solve the problem
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        //std::cout << summary.BriefReport() << "\n";
        r= abs(r);
        std::cout << "Before   x: " << x << " y: " << y << " r: " << r <<" residual: "<<summary.final_cost<<"\n";
        //push the result to output vector
        Point3f temp3d(x, y, r);

        if(r>=tree_radius_min && r<=tree_radius_max && summary.final_cost<=tree_residual){
            //TODO:VISUAL tree and PUBLISH CENTER
            //Final good tree output
            std::cout << "Confirmed   x: " << x << " y: " << y << " r: " << r <<"\n";
        }else{
            //UNVISUAL NOISE
            for(int k = 0; k < currentClusterIdx[j].size(); k++){
                dataset.channels[DBscanDriver::type].values[currentClusterIdx[j][k]] = DBscanDriver::strange;
                dataset.channels[DBscanDriver::cluster].values[currentClusterIdx[j][k]] = 0.0;
            }
        }
    }
    tree_cloud_pub.publish(dataset);
}



void point_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    //Convert sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
    sensor_msgs::PointCloud output;
    sensor_msgs::convertPointCloud2ToPointCloud(*input, output);
    cloud_pub.publish(output);

//    //test PCA
//    PCA_Eigen(output);

//    //test VFH
//    Voxel_Filter_Hash(output);

//    //test KDTree
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    KD_TREE_NN(output);
//    endTime = clock();//计时结束
//    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test Octree
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    OctreeDriver oldDriver;
//    oldDriver.octreeNNdemo(output);
//    endTime = clock();//计时结束
//    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test DBscan
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    DBscanDriver oldDriver;
//    oldDriver.dbscanClustering(output);
//    tree_cloud_pub.publish(oldDriver.PCLforOutput);
//    endTime = clock();//计时结束
//    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test Kmeans
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    tree_cloud_pub.publish(Kmeans(output));
//    endTime = clock();//计时结束
//    cout << "The K-means run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test gmmEM
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    tree_cloud_pub.publish(gmmEM(output));
//    endTime = clock();//计时结束
//    cout << "The gmmEM run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test spectralClustering
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    tree_cloud_pub.publish(spectralClustering(output));
//    endTime = clock();//计时结束
//    cout << "The spectralClustering run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    //test RANSAC BSCAN
    clock_t startTime,endTime;
    startTime = clock();//计时开始
    ransacDriver oldDriver;

    oldDriver.setGroundZMaxAndMin(groundZMax, groundZMin);
    oldDriver.setInlinerRatio(inlierRatio);
    oldDriver.setSampleNum(sampleNum);
    oldDriver.setConfidence(confidence);
    oldDriver.setInlinerThreshold(inlinerThreshold);
    oldDriver.setRatioCondition(ratioCondition);
    oldDriver.setUpperBorder(upperBorder);

    oldDriver.groundRemove(output);
    //tree_cloud_pub.publish(oldDriver.PCLforOutput);
    endTime = clock();//计时结束
    cout << "The RANSAC run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    for(int i = 0; i < oldDriver.PCLforOutput.points.size(); i++){
        if(oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values[i] == 1){
            oldDriver.PCLforOutput.points.erase(oldDriver.PCLforOutput.points.begin() + i);
            oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.erase(oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.begin() + i);
            i--;
        }
    }


    for(int i = 0; i < oldDriver.PCLforOutput.points.size(); i++){
        if(oldDriver.PCLforOutput.points[i].z >= 2.3 + height_max){
            oldDriver.PCLforOutput.points.erase(oldDriver.PCLforOutput.points.begin() + i);
            oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.erase(oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.begin() + i);
            i--;
        }
    }

    sensor_msgs::PointCloud dataset = oldDriver.PCLforOutput;

//    //REAL DBSCAN
//    int counter = 0;
//    sensor_msgs::PointCloud dataset;
//    dataset.header = output.header;
//    for(auto pt_iter : output.points){
//        if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) <= (distance_max * distance_max))
//            if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) >= (distance_max * distance_min))
//                if(pt_iter.z >= height_min)
//                    if(pt_iter.z <= height_max){
//                        counter++;
//                        dataset.points.push_back(pt_iter);
//                    }
//    }
//    cout<<"dataset_size: "<<dataset.points.size() << endl;
//    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;

    DBSCAN(dataset,EPS,MinPts);
}





int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "dbscaner");
    ros::NodeHandle nh_;
    string tree_pt,scan_name;

    ros::param::get("~scan_range",scan_range);
    ros::param::get("~scan_number_360",scan_num360);
    ros::param::get("~tree_point",tree_pt);
    ros::param::get("~scan_topic_name",scan_name);

    ros::param::get("~EPS",EPS);
    ros::param::get("~MinPts",MinPts);
    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;
    ros::param::get("~tree_residual",tree_residual);
    ros::param::get("~tree_radius_max",tree_radius_max);
    ros::param::get("~tree_radius_min",tree_radius_min);

    ros::param::get("~min_cluster",min_cluster);
    ros::param::get("~distance_max",distance_max);
    ros::param::get("~distance_min",distance_min);
    ros::param::get("~height_max",height_max);
    ros::param::get("~height_min",height_min);

    ros::param::get("~groundZMax",groundZMax);
    ros::param::get("~groundZMin",groundZMin);
    ros::param::get("~inlierRatio",inlierRatio);
    ros::param::get("~confidence",confidence);
    ros::param::get("~inlinerThreshold",inlinerThreshold);
    ros::param::get("~ratioCondition",ratioCondition);
    ros::param::get("~upperBorder",upperBorder);
    ros::param::get("~sampleNum",sampleNum);



    cout<<"scan_range:"<<scan_range<<endl;
    cout<<"scan_number_360:"<<scan_num360<<endl;
    cout<<"tree_point:"<<tree_pt<<endl;
    cout<<"scan_topic_name:"<<scan_name<<endl;
    circle_pub = nh_.advertise<sensor_msgs::PointCloud2>(tree_pt,1);
    ros::Subscriber scan_sub = nh_.subscribe(scan_name, 1, point_callback);

    //test
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud1", 1);
    tree_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("tree_center", 1);
    tree_visual_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("tree_cloud_visual", 1);



    ros::spin();
    return 0;
}