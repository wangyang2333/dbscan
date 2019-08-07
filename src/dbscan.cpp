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

double scan_num360;
double scan_range;



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
    centers.ranges.resize(1000);

    //---------------remove little cluster----------
    for(int f = 0; f<data.size(); f++)
    {
        if(data[f].size()< 10) data.erase(data.begin()+f);
    }
    //----------------------------------------------

    cout<<"cluster size:"<<data.size()<<endl;
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
            //normalize rou and sita to the ratio it occupies in full range.
            point temp_pt = point(counter/scan_num360, scan_r/scan_range, counter);
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
    string tree_pt,scan_name;

    ros::param::get("~scan_range",scan_range);
    ros::param::get("~scan_number_360",scan_num360);
    ros::param::get("~tree_point",tree_pt);
    ros::param::get("~scan_topic_name",scan_name);
    cout<<"scan_range:"<<scan_range<<endl;
    cout<<"scan_number_360:"<<scan_num360<<endl;
    cout<<"tree_point:"<<tree_pt<<endl;
    cout<<"scan_topic_name:"<<scan_name<<endl;
    circle_pub = nh_.advertise<sensor_msgs::LaserScan>(tree_pt,1);
    ros::Subscriber scan_sub = nh_.subscribe(scan_name, 1, scan_callback);
    ros::spin();
    return 0;
}
