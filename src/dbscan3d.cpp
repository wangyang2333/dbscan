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
#include <thread>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

//#include "kd_tree_nn.h"
#include "octree_nn.h"
//#include "kmeans.h"
//#include "gmm_EM.h"
//#include "spectral_clustering.h"

using namespace cv;
using namespace std;

std::mutex scan_lock;
double scan_num360;
double scan_range;
double min_cluster;
double distance_max;
double distance_min;
double height_max;
double height_min;
int MinPts;
double EPS, tree_residual, tree_radius_max, tree_radius_min;

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
sensor_msgs::LaserScan centers ;
void DBSCAN(vector<point> dataset,double Eps,int MinPts){//按照xy密度来进行聚类。
    int len = dataset.size();
    //calculate pts
    //cout<<"calculate pts"<<endl;
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    ROS_INFO("Data Point size : %d", int(dataset.size()));
    for(int i=0;i<len;i++){
        for(int j=i+1;j<len;j++){
            if(dataset[i].pts >= MinPts){
                break;
            }
            if(squareDistance(dataset[i],dataset[j])<Eps)
                dataset[i].pts++;
            dataset[j].pts++;
            //END EARILER TO SAVE COMPUTATION
        }
    }
    endTime = clock();//计时结束
    cout << "The Brute-Force search time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


    //core point
    //cout<<"core point "<<endl;
    vector<point> corePoint;
    for(int i=0;i<len;i++){
        if(dataset[i].pts>=MinPts) {
            dataset[i].pointType = 3;
            corePoint.push_back(dataset[i]);
        }
    }
    endTime = clock();//计时结束
    cout << "The CorePts Find time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    //cout<<"joint core point"<<endl;
    //joint core point
    ROS_INFO("Core Point size : %d", int(corePoint.size()));
    for(int i=0;i<corePoint.size();i++){
        for(int j=i+1;j<corePoint.size();j++){
            if(squareDistance(corePoint[i],corePoint[j])<Eps){
                corePoint[i].corepts.push_back(j);
                corePoint[j].corepts.push_back(i);
            }
        }
    }
    endTime = clock();//计时结束
    cout << "The CorePts Combining time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    for(int i=0;i<corePoint.size();i++){
        stack<point*> ps;
        if(corePoint[i].visited == 1) continue;
        ps.push(&corePoint[i]);
        point *v;
        while(!ps.empty()){
            v = ps.top();
            v->visited = 1;
            ps.pop();
            for(int j=0;j<v->corepts.size();j++){
                if(corePoint[v->corepts[j]].visited==1) continue;
                corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
                corePoint[v->corepts[j]].visited = 1;
                ps.push(&corePoint[v->corepts[j]]);
            }
        }
    }
    endTime = clock();//计时结束
    cout << "The CorePts join time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    //cout<<"border point,joint border point to core point"<<endl;
    //border point,joint border point to core point
    for(int i=0;i<len;i++){
        if(dataset[i].pointType==3) continue;
        for(int j=0;j<corePoint.size();j++){
            if(squareDistance(dataset[i],corePoint[j])<Eps) {
                dataset[i].pointType = 2;
                dataset[i].cluster = corePoint[j].cluster;
                break;
            }
        }
    }
    endTime = clock();//计时结束
    cout << "The BorderPts join time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    //Number Each Data
    int temp_cluster = 1;
    if(!corePoint.empty()){
        for(int i=0;i<corePoint.size()-1 ;i++){
            if(corePoint[i].cluster == corePoint[i+1].cluster){
                corePoint[i].cluster = temp_cluster;
            }
            else{
                corePoint[i].cluster = temp_cluster;
                temp_cluster++;
            }
        }
        //cout<<"end number"<<endl;
        corePoint[corePoint.size()-1].cluster = temp_cluster;
    }

    //Push Data Into Cluster//
    vector<vector<point>> data;
    vector<point> temp_vec;
    if(!corePoint.empty()){
        //cout<<"begin push"<<endl;
        for(int i=0;i<corePoint.size()-1 ;i++){
            if(corePoint[i].cluster == corePoint[i+1].cluster){
                temp_vec.push_back(corePoint[i]);
            }
            else{
                temp_vec.push_back(corePoint[i]);
                data.push_back(temp_vec);
                temp_vec.clear();
            }

        }
        temp_vec.push_back(corePoint[corePoint.size()-1]);
        data.push_back(temp_vec);
    }

    //remove little cluster
    for(int f = 0; f<data.size(); f++)
    {
        if(data[f].size()< min_cluster){
            //cout<<"Do erase a cluster with "<<data[f].size()<<" elements\n";
            data.erase(data.begin()+f);
            f--;
        }


    }


    //New output:
    for(int i=0; i<data.size(); i++){
        for(int j=0; j<data[i].size(); j++){
            //cout<<"pts: "<<data[i][j].x<<","<<data[i][j].y<<","<<i<<"\n";
        }
    }
    centers.ranges.clear();
    centers.ranges.resize(1000);
    //cout<<"cluster size:"<<data.size()<<endl;
    endTime = clock();//计时结束
    cout << "The Cluster finish time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


    //三维地卡尔坐标系中的ceres优化树心轴。
    //对每个cluster进行优化的for
    std::vector<Point3f> temp_tree_pt;

    std::vector<Point3f> tree_visual;
    std::vector<std::vector<Point3f>> tree_visual_full;


    for(int j = 0; j < data.size() ;j ++) {
        //----------------ceres test---------------

        //优化初始值把类中第一个点的位置复制给他
        double x = data[j][0].x;
        double y = data[j][0].y;
        double r = 0.15;

        ceres::Problem problem;
        for (int i = 0; i < data[j].size() / 2; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<circleResidual, 1, 1, 1, 1>(
                            new circleResidual(data[j][i].x, data[j][i].y)),
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
        std::cout << "Before   x: " << x << " y: " << y << " r: " << r <<"\n";
        //push the result to output vector
        Point3f temp3d(x, y, r);
        tree_visual.clear();
        if(r>=tree_radius_min && r<=tree_radius_max && summary.final_cost<=tree_residual){
            temp_tree_pt.push_back(temp3d);

            //Do visualization,搞一个向量的向量存Point3f，每个小向量里面是同一棵树的点云
            for(int i = 0; i < data[j].size(); ++i){
                Point3f temp3d(data[j][i].x, data[j][i].y, data[j][i].z);
                tree_visual.push_back(temp3d);
            }
            tree_visual_full.push_back(tree_visual);

            //Final good tree output
            std::cout << "Confirmed   x: " << x << " y: " << y << " r: " << r <<"\n";
        }

    }
    endTime = clock();//计时结束
    cout << "The Optimization time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    sensor_msgs::PointCloud tree_cloud;
    tree_cloud.header.frame_id = "velodyne";
    tree_cloud.points.resize(temp_tree_pt.size());
    tree_cloud.channels.resize(1);
    tree_cloud.channels[0].name = "intensities";
    tree_cloud.channels[0].values.resize(temp_tree_pt.size());
    int count = 0;
    for(auto gdlmk : temp_tree_pt)
    {
        tree_cloud.points[count].x = gdlmk.x;
        tree_cloud.points[count].y = gdlmk.y;
        tree_cloud.points[count].z = gdlmk.z;
        tree_cloud.channels[0].values[count] = count;
        count++;
    }
    tree_cloud_pub.publish(tree_cloud);

    sensor_msgs::PointCloud tree_visual_cloud;
    tree_visual_cloud.header.frame_id = "velodyne";
    //tree_visual_cloud.points.resize(tree_visual.size()*tree_visual_full.size());
    tree_visual_cloud.channels.resize(1);
    tree_visual_cloud.channels[0].name = "intensities";
    //tree_visual_cloud.channels[0].values.resize(tree_visual.size()*tree_visual_full.size());
    int count_v = 0;
    for(auto vec : tree_visual_full){
        for(auto pts : vec)
        {
            geometry_msgs::Point32 temp_pts_converter;
            temp_pts_converter.x = pts.x;
            temp_pts_converter.y = pts.y;
            temp_pts_converter.z = pts.z;
            tree_visual_cloud.points.push_back(temp_pts_converter);
            tree_visual_cloud.channels[0].values.push_back(count_v);
        }
        count_v++;
    }
    tree_visual_cloud_pub.publish(tree_visual_cloud);
    endTime = clock();//计时结束
    cout << "The Data Visualization time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

}

void PCA_Eigen(sensor_msgs::PointCloud& PCL){
    cout<<"Begin global PCA"<<endl;
    double xmean, ymean, zmean;
    for (auto pts : PCL.points)
    {
        xmean += pts.x;
        ymean += pts.y;
        zmean += pts.z;
    }
    xmean = xmean / PCL.points.size();
    ymean = ymean / PCL.points.size();
    zmean = zmean / PCL.points.size();
    Eigen::Matrix<double, -1, -1> PCA_X (PCL.points.size(),3);
    //Insert point into the Matrix
    for(int i=0; i<PCL.points.size(); i++){
        //in iteration the i < PCL.point.seze()
        PCA_X(i,0) = PCL.points[i].x - xmean;
        PCA_X(i,1) = PCL.points[i].y - ymean;
        PCA_X(i,2) = PCL.points[i].z - zmean;
    }
    Eigen::Matrix<double, 3, 3> PCA_XXT;
    PCA_XXT = PCA_X.transpose() * PCA_X;
    //Eigen::Vector3d PCA_Eigenvalue = PCA_XXT.eigenvalues();
    Eigen::EigenSolver<Eigen::Matrix3d> es(PCA_XXT);
    //Use Dynamic and Complex matrix, Beacuse the eigenvalue can be complex.
    Eigen::MatrixXcd Lamda = es.eigenvalues();
    Eigen::MatrixXcd U = es.eigenvectors();
    //The principle vector are the columns of Ur
    cout << "The original Matrix" << endl << PCA_XXT <<endl;
    cout << "The eigenvalues Lamdas are:" << endl << Lamda << endl;
    cout << "The eigenvectors matrix U are:" << endl << U << endl;
}

void Voxel_Filter_Hash(sensor_msgs::PointCloud& PCL){
    cout<<"Begin VFH"<<endl;
    double xmean, ymean, zmean;
    for (auto pts : PCL.points)
    {
        xmean += pts.x;
        ymean += pts.y;
        zmean += pts.z;
    }
    xmean = xmean / PCL.points.size();
    ymean = ymean / PCL.points.size();
    zmean = zmean / PCL.points.size();
    Eigen::Matrix<double, -1, -1> VFH_X (PCL.points.size(),3);
    //Insert point into the Matrix
    for(int i=0; i<PCL.points.size(); i++){
        //in iteration the i < PCL.point.seze()
        VFH_X(i,0) = PCL.points[i].x-xmean;
        VFH_X(i,1) = PCL.points[i].y-ymean;
        VFH_X(i,2) = PCL.points[i].z-zmean;
    }
    //Compute Dx Dy Dz
    Eigen::Vector3d Max_Cor = VFH_X.colwise().maxCoeff();
    Eigen::Vector3d Min_Cor = VFH_X.colwise().minCoeff();
    double VG_size = 0.2;
    int Dx, Dy, Dz;// int Dx Dy Dz may cause some problems. Come back later!
    Dx = (Max_Cor(0) - Min_Cor(0)) / VG_size +1;
    Dy = (Max_Cor(1) - Min_Cor(1)) / VG_size +1;
    Dz = (Max_Cor(2) - Min_Cor(2)) / VG_size +1;
    //cout << "Dx: " << Dx << " Dy: " << Dy << " Dz: " << Dz << endl;

    //Compute Voxel Index for Each PTS and Push them into my Hash Table
    int PTS_Num_after_VF = 500;
    std::vector<  pair<std::vector<geometry_msgs::Point32 >,long int> > Hash_table;
    sensor_msgs::PointCloud PCL_after_VF;
    PCL_after_VF.header.frame_id = "velodyne";
    PCL_after_VF.channels.resize(1);
    PCL_after_VF.channels[0].name = "intensities";
    Hash_table.resize(PTS_Num_after_VF);
    for(int i=0; i<PCL.points.size(); i++){
        int hx = floor((VFH_X(i,0) -Min_Cor(0))/VG_size);
        int hy = floor((VFH_X(i,1) -Min_Cor(1))/VG_size);
        int hz = floor((VFH_X(i,2) -Min_Cor(2))/VG_size);
        //cout << "hx: " << hx << " hy: " << hy << " hz: " << hz << endl;
        //Hash PTS to 100 voxels.
        //Careful this may cause over flow

        geometry_msgs::Point32 current_pts;
        current_pts.x = VFH_X(i,0) + xmean;
        current_pts.y = VFH_X(i,1) + ymean;
        current_pts.z = VFH_X(i,2) + zmean;
        long int h = (hx + hy * Dx + hz * Dx * Dy);
        long int hash_num = h % PTS_Num_after_VF; //0~PTS_Num_VF-1
        if(Hash_table[hash_num].first.empty()){
            Hash_table[hash_num].first.push_back(current_pts);
            Hash_table[hash_num].second = h;
        }
        else{
            if(h == Hash_table[hash_num].second){
                Hash_table[hash_num].first.push_back(current_pts);
            }
            else{
                //Handle Conflict: Compute old mean and push into new PCL
                geometry_msgs::Point32 mean_pts;
                for(auto pts : Hash_table[hash_num].first){
                    mean_pts.x += pts.x;
                    mean_pts.y += pts.y;
                    mean_pts.z += pts.z;
                }
                mean_pts.x = mean_pts.x / Hash_table[hash_num].first.size();
                mean_pts.y = mean_pts.y / Hash_table[hash_num].first.size();
                mean_pts.z = mean_pts.z / Hash_table[hash_num].first.size();
                PCL_after_VF.points.push_back(mean_pts);
                //Clear and pushback new pts
                Hash_table[hash_num].first.clear();
                Hash_table[hash_num].first.push_back(current_pts);
                Hash_table[hash_num].second = h;
            }
        }
    }
    cout<<endl<<"i ama here"<<endl;
    for( auto voxel : Hash_table){
        if(!voxel.first.empty())
        {
            geometry_msgs::Point32 mean_pts2;
            for(auto pts : voxel.first){
                mean_pts2.x += pts.x;
                mean_pts2.y += pts.y;
                mean_pts2.z += pts.z;
            }
            mean_pts2.x = mean_pts2.x / voxel.first.size();
            mean_pts2.y = mean_pts2.y / voxel.first.size();
            mean_pts2.z = mean_pts2.z / voxel.first.size();
            PCL_after_VF.points.push_back(mean_pts2);
        }
        else{
            cout<<endl<<"Empty Hash Bucket!"<<endl;
        }
    }
    tree_visual_cloud_pub.publish(PCL_after_VF);
    //cout << endl << VFH_X << endl;
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

    //test Octree
    clock_t startTime,endTime;
    startTime = clock();//计时开始
    OctreeDriver oldDriver;
    oldDriver.OCTREE_NN(output);
    endTime = clock();//计时结束
    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

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

//    //Convert sensor_msgs::PointCloud to my_own::point
//    int counter = 0;
//    std::vector<point> dataset;
//    for(auto pt_iter : output.points){
//        if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) <= (distance_max * distance_max))
//            if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) >= (distance_max * distance_min))
//                if(pt_iter.z >= height_min)
//                    if(pt_iter.z <= height_max){
//                        point temp_pt = point(pt_iter.x, pt_iter.y, pt_iter.z, counter);
//                        counter++;
//                        dataset.push_back(temp_pt);
//                    }
//    }
//    cout<<"dataset_size: "<<dataset.size() << endl;
//    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;
//    DBSCAN(dataset,EPS,MinPts);
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