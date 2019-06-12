//
// Created by xcy on 19-6-11.
//
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
#include <geometry_msgs/PointStamped.h>

#include <dlib/svm_threaded.h>
#include <dlib/optimization/max_cost_assignment.h>
#ifndef SRC_TREE_TRACKER_H
#define SRC_TREE_TRACKER_H
using namespace cv;
using namespace std;
using namespace dlib;

class tree{
public:
    double sita;
    double rou;
    double tree_id;
    tree(double x_,double y_, double id_){
        sita = x_;
        rou = y_;
        tree_id = id_;
    }
};

class tree_tracker{
public:
    std::vector<tree> last_trees;
    std::vector<tree> this_track;
    std::vector<tree> this_trees;


    void track_tree();
    double norm_distance(tree ta, tree tb);
    int find_bad_tree(std::vector<tree> tree_more , std::vector<tree> tree_less);

    bool first_track_flag;

    tree_tracker(){
        first_track_flag = true;
    }
};

double tree_tracker::norm_distance(tree ta, tree tb){
    return (abs(ta.sita - tb.sita) + abs(ta.rou - tb.rou));
}

int tree_tracker::find_bad_tree(std::vector<tree> tree_more , std::vector<tree> tree_less){
    double max_dist = 0 ;
    int max_index = 0 ;
    for(int i = 0; i<tree_more.size(); i++){
        double min = 99999999.9;
        for(int j =0; j<tree_less.size(); j++){
            if(norm_distance(tree_more[i],tree_less[j])*100000 < min){
                min = norm_distance(tree_more[i],tree_less[j])*100000;
            }

        }
        if(min > max_dist){
            max_index = i;
            max_dist = min;
            cout<<"index is"<<max_index<<endl;
            cout<<"sum is :"<<max_dist<<endl;
            cout<<"i = "<<i<<endl;

        }
    }
    cout<<"new tree:"<<max_index<<endl;
    return max_index;
}

void tree_tracker::track_tree(){
    /*
    for(auto t: this_trees){
        cout<<"this rou:"<<t.rou<<"  this sita:"<<t.sita<<endl;
    }
    for(auto t: last_trees){
        cout<<"this rou:"<<t.rou<<"  this sita:"<<t.sita<<endl;
    }
     */
    if(this_trees.size()==this_track.size())
    {
        dlib::matrix<long int> cost(this_trees.size(),this_track.size());
        /*cost = int(0.3673329*10000), 2,
                5, 3,
                4, 5;(3,2)*/

        for(int tre = 0; tre < this_trees.size(); tre++){
            for(int tra = 0; tra < this_track.size(); tra++){
                cost(tre,tra) = -norm_distance(this_trees[tre],this_track[tra])*100000;
                cout<<"this tree cost :"<<cost(tre,tra)<<endl;
            }

        }
        // To find out the best assignment of people to jobs we just need to call this function.
        std::vector<long> assignment = max_cost_assignment(cost);

        // This prints optimal assignments:  [2, 0, 1] which indicates that we should assign
        // the person from the first row of the cost matrix to job 2, the middle row person to
        // job 0, and the bottom row person to job 1.
        for (unsigned int i = 0; i < assignment.size(); i++)
            cout << assignment[i] << std::endl;

        // This prints optimal cost:  16.0
        // which is correct since our optimal assignment is 6+5+5.
        cout << "optimal cost: " << assignment_cost(cost, assignment) << endl;

        for(int i = 0; i < this_trees.size(); i++){
            this_track[assignment[i]].sita = this_trees[i].sita;
            this_track[assignment[i]].rou = this_trees[i].rou;
            printf("tracked tree id: %lf    rou: %lf   sita: %lf \n",this_track[assignment[i]].tree_id, this_track[assignment[i]].rou,this_track[assignment[i]].sita);
        }
    }
    else if(this_trees.size()>this_track.size()){
        for(int i = 0 ; i < this_trees.size() - this_track.size() ; i++){
            this_track.push_back(this_trees[find_bad_tree(this_trees,this_track )]);
        }

    }
    else if(this_trees.size()<this_track.size()){
        cout<<"remove old tree"<<endl;
        this_track.erase(this_track.begin()+find_bad_tree(this_track,this_trees));
    }
}





#endif //SRC_TREE_TRACKER_H
