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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//------------------------------------
#include <sensor_msgs/LaserScan.h>




using namespace cv;
using namespace std;

std::mutex scan_lock;



class point{
public:
	float x;
	float y;
	int cluster=0;
	int pointType=1;//1 noise 2 border 3 core
	int pts=0;//points in MinPts 
	vector<int> corepts;
	int visited = 0;
	point (){}
	point (float a,float b,int c){
		x = a;
		y = b;
		cluster = c;
	}
};

float squareDistance(point a,point b){
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

struct circleResidual {
    circleResidual(double sitai, double roui)
            : sitai_(sitai),roui_(roui) {}
    template <typename T> bool operator()(const T* const rouR,
                                          const T* const r,
                                          const T* const faiR,
                                          T* residual) const {
        residual[0] = r[0]*r[0] - rouR[0]*rouR[0] - roui_*roui_ + 2.0 * rouR[0] * roui_ * cos(faiR[0]-sitai_);
        return true;
    }
private:
    const double sitai_;
    const double roui_;
};

ros::Publisher circle_pub;
sensor_msgs::LaserScan centers ;

void DBSCAN(vector<point> dataset,float Eps,int MinPts){
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
    centers.ranges.resize(640);

    //---------------remove noise again----------
    for(int f = 0; f<=data.size(); f++)
    {
        if(data[f].size()<=10) data.erase(data.begin()+f);
    }

    for(int j = 0; j < data.size() ;j ++){
        //----------------ceres test---------------
        double rouR = 2.2689;
        double faiR = 2.2689;
        double r = 0.14;

        ceres::Problem problem;
        for (int i = 0; i < data[j].size()/2; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<circleResidual, 1, 1 , 1, 1>(
                            new circleResidual(data[j][2 * i]/180.0*260.0*M_PI, data[j][2 * i + 1]*8.0)),
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
        std::cout << "Initial rouR: " << 0.0 << " faiR: " << 0.0 << " r: " << 0.0 <<"\n";
        std::cout << "Final   rouR: " << abs(rouR) << " faiR: " << faiR << " r: " << r <<"\n";
        if(rouR > 0){
            cout<<"fai_int:"<<int(faiR*180.0/260.0/M_PI*640.0)%886<<endl;
            if(int(faiR*180.0/260.0/M_PI*640.0)%886 <640 && int(faiR*180.0/260.0/M_PI*640.0)%886>=0)
            centers.ranges[int(faiR*180.0/260.0/M_PI*640.0)%886] = float(rouR);//最优解极径可正可负
        }
        else{
            cout<<"fai_int"<<(int(faiR*180.0/260.0/M_PI*640.0)+443)%886<<endl;
            if((int(faiR*180.0/260.0/M_PI*640.0)+443)%886 <640 && int(int(faiR*180.0/260.0/M_PI*640.0)+443)%886>=0)
            centers.ranges[(int(faiR*180.0/260.0/M_PI*640.0)+443)%886] = -float(rouR);
        }
        //----------------ceres test---------------
    }
    circle_pub.publish(centers);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    scan_lock.lock();
    centers.header.stamp=scan->header.stamp;
    centers.header.frame_id=scan->header.frame_id;
    centers.angle_min=scan->angle_min;
    centers.angle_max=scan->angle_max;
    centers.angle_increment=scan->angle_increment;
    centers.time_increment=scan->time_increment;
    centers.range_min=scan->range_min;
    centers.range_max=scan->range_max;
    centers.ranges.resize(scan->ranges.size());
    vector<point> dataset;
    int counter = 0;
    for(auto  scan_r:scan->ranges ){
        if(scan_r >= 0.2 && scan_r<=8.0){
            point temp_pt = point(counter/640.0,scan_r/8.0,counter);
            dataset.push_back(temp_pt);
        }
        counter++;
    }
    cout<<"dataset_size: "<<dataset.size() << endl;
    DBSCAN(dataset,0.02,5);
    scan_lock.unlock();
}



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "dbscaner");
    ros::NodeHandle nh_;
    circle_pub = nh_.advertise<sensor_msgs::LaserScan>("/scan2",1);
    ros::Subscriber scan_sub = nh_.subscribe("/base_scan", 1, scan_callback);
    ros::spin();
    return 0;
}
