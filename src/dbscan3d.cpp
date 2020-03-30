/*All right reserved.
 *if tf in rviz can't display, run a roscore first.
 */


#include <queue>
#include <sensor_msgs/Imu.h>
#include <ros/forwards.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <mutex>
#include <condition_variable>
#include <thread>
#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//------------------------------------
#include <sensor_msgs/LaserScan.h>


/*
 * We use dbscan in 3d-coordinate.
 *
 * Then we creat a function to compute rou and sita of every point in class.
 *
 *
 *
 *
 *
 *
 */


using namespace cv;
using namespace std;

std::mutex scan_lock;

double scan_num360;
double scan_range;

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

sensor_msgs::LaserScan centers ;
void DBSCAN(vector<point> dataset,double Eps,int MinPts){//按照xy密度来进行聚类。
    int len = dataset.size();
    //calculate pts
    cout<<"calculate pts"<<endl;
    for(int i=0;i<len;i++){
        for(int j=i+1;j<len;j++){
            if(squareDistance(dataset[i],dataset[j])<Eps)
                dataset[i].pts++;
            dataset[j].pts++;
        }
    }
    //core point
    cout<<"core point "<<endl;
    vector<point> corePoint;
    for(int i=0;i<len;i++){
        if(dataset[i].pts>=MinPts) {
            dataset[i].pointType = 3;
            corePoint.push_back(dataset[i]);
        }
    }
    cout<<"joint core point"<<endl;
    //joint core point
    for(int i=0;i<corePoint.size();i++){
        for(int j=i+1;j<corePoint.size();j++){
            if(squareDistance(corePoint[i],corePoint[j])<Eps){
                corePoint[i].corepts.push_back(j);
                corePoint[j].corepts.push_back(i);
            }
        }
    }
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
    cout<<"border point,joint border point to core point"<<endl;
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



	//-------------------number data-----------------//
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
        cout<<"end number"<<endl;
        corePoint[corePoint.size()-1].cluster = temp_cluster;
    }

    //-----------push data---------------------//
    vector<vector<double>> data;
    vector<double> temp_vec;
    if(!corePoint.empty()){
        cout<<"begin push"<<endl;
        for(int i=0;i<corePoint.size()-1 ;i++){
            if(corePoint[i].cluster == corePoint[i+1].cluster){
                temp_vec.push_back(corePoint[i].x);
                temp_vec.push_back(corePoint[i].y);
            }
            else{
                temp_vec.push_back(corePoint[i].x);
                temp_vec.push_back(corePoint[i].y);
                data.push_back(temp_vec);
                temp_vec.clear();
            }

        }
        temp_vec.push_back(corePoint[corePoint.size()-1].x);
        temp_vec.push_back(corePoint[corePoint.size()-1].y);
        data.push_back(temp_vec);
    }

	//output
    for(int i=0;i<len;i++){
        if(dataset[i].pointType == 2)
            cout<<"border:"<<dataset[i].x<<","<<dataset[i].y<<","<<dataset[i].cluster<<"\n";
    }
    for(int i=0;i<corePoint.size();i++){
        cout<<"core:"<<corePoint[i].x<<","<<corePoint[i].y<<","<<corePoint[i].cluster<<"\n";
    }
    centers.ranges.clear();
    centers.ranges.resize(1000);

    //---------------remove little cluster----------//TODO: change the size of dbscan
    for(int f = 0; f<data.size(); f++)
    {
        if(data[f].size()< 10) data.erase(data.begin()+f);
    }
    //----------------------------------------------

    cout<<"cluster size:"<<data.size()<<endl;

    //三维地卡尔坐标系中的ceres优化树心轴。
    //对每个cluster进行优化的for
    std::vector<Point3f> temp_tree_pt;
    for(int j = 0; j < data.size() ;j ++) {
        //----------------ceres test---------------

        //优化初始值把类中第一个点的位置复制给他
        double x = data[j][0];
        double y = data[j][1];
        double r = 0.15;

        ceres::Problem problem;
        for (int i = 0; i < data[j].size() / 2; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<circleResidual, 1, 1, 1, 1>(
                            new circleResidual(data[j][2 * i], data[j][2 * i + 1])),
                    //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &x, &y, &r);
        }
        //solve the problem
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "Final   x: " << x << " y: " << y << " r: " << r <<"\n";
        //push the result to output vector
        Point3f temp3d(x, y, r);
        temp_tree_pt.push_back(temp3d);
    }



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



    /*
    for(int j = 0; j < data.size() ;j ++){
        //----------------ceres test---------------
        double faiR = data[j][0]*2*M_PI;  //inverse normalization and transfer from deg. to rad.
        double rouR = data[j][1]*scan_range;
        double r = 0.2;

        ceres::Problem problem;
        for (int i = 0; i < data[j].size()/2; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<circleResidual, 1, 1 , 1, 1>(
                            new circleResidual(data[j][2 * i]*2*M_PI, data[j][2 * i + 1]*scan_range)),
                            //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &rouR, &r, &faiR);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "Final   rouR: " << abs(rouR) << " faiR: " << faiR << " r: " << r <<"\n";
        if(r>=tree_radius_min && r<=tree_radius_max && summary.final_cost<=tree_residual){//control the radius
            if(rouR > 0){//The rouR can be positive or negative
                if( int(faiR/2.0/M_PI*scan_num360)%int(scan_num360)>=0){//the faiR have a 2pi cycle. %886 for 2*pi
                    centers.ranges[(int(faiR/2.0/M_PI*scan_num360))%int(scan_num360)] = float(rouR);
                    cout<<">0's laser num: "<<(int(faiR/2.0/M_PI*scan_num360))%int(scan_num360)<<endl;
                }//However, the result after %886 can also be negative.
                else{//If negative after %886, We + 886 after %886
                    centers.ranges[(int(faiR/2.0/M_PI*scan_num360))%int(scan_num360)+int(scan_num360)] = float(rouR);
                    cout<<">0's laser num: "<<(int(faiR/2.0/M_PI*scan_num360))%int(scan_num360)+int(scan_num360)<<endl;
                }
            }
            else{
                if(int(faiR/2/M_PI*scan_num360+scan_num360/2.0)%int(scan_num360)>=0){
                    centers.ranges[int(faiR/2.0/M_PI*scan_num360+scan_num360/2.0)%int(scan_num360)] = -float(rouR);
                    cout<<"<0's laser num: "<<int(faiR/2.0/M_PI*scan_num360+scan_num360/2.0)%int(scan_num360)<<endl;
                }
                else{
                    centers.ranges[int(faiR/2.0/M_PI*scan_num360+scan_num360/2.0)%int(scan_num360)+int(scan_num360)] = -float(rouR);
                    cout<<"<0's laser num: "<<int(faiR/2.0/M_PI*scan_num360+scan_num360/2.0)%int(scan_num360)+int(scan_num360)<<endl;
                }
            }
        }
        //----------------ceres test---------------
    }
    circle_pub.publish(centers);

    */
}


//test


void point_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    //Convert sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
    sensor_msgs::PointCloud output;
    sensor_msgs::convertPointCloud2ToPointCloud(*input, output);
    cloud_pub.publish(output);

    //Convert sensor_msgs::PointCloud to my_own::point
    double distance_max = 10.0;
    double distance_min = 0.3;
    double height_max = 1.8;
    double height_min = 0.4;
    int counter = 0;
    std::vector<point> dataset;
    for(auto pt_iter : output.points){
        if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) <= (distance_max * distance_max))
            if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) >= (distance_max * distance_min))
                if(pt_iter.z >= height_min)
                    if(pt_iter.z <= height_max){
                        point temp_pt = point(pt_iter.x, pt_iter.y, pt_iter.z, counter);
                        counter++;
                        dataset.push_back(temp_pt);
                    }
    }
    cout<<"dataset_size: "<<dataset.size() << endl;
    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;
    DBSCAN(dataset,EPS,MinPts);






    /*
    scan_lock.lock();
    vector<point> dataset;
    int counter = 0;
    for(auto  point_r:point->data.size() ){
        if(scan_r >= 0.2 && scan_r<=8.0){
            //normalize rou and sita to the ratio it occupies in full range.
            point temp_pt = point(counter/scan_num360, scan_r/scan_range, counter);
            dataset.push_back(temp_pt);
        }
        counter++;
    }
    cout<<"dataset_size: "<<dataset.size() << endl;
    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;
    DBSCAN(dataset,EPS,MinPts);
    scan_lock.unlock();
    */
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

    cout<<"scan_range:"<<scan_range<<endl;
    cout<<"scan_number_360:"<<scan_num360<<endl;
    cout<<"tree_point:"<<tree_pt<<endl;
    cout<<"scan_topic_name:"<<scan_name<<endl;
    circle_pub = nh_.advertise<sensor_msgs::PointCloud2>(tree_pt,1);
    ros::Subscriber scan_sub = nh_.subscribe(scan_name, 1, point_callback);

    //test
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud1", 1);
    tree_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("tree_cloud1", 1);



    ros::spin();
    return 0;
}
