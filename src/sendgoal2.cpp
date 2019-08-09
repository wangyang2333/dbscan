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
#include <geometry_msgs/PointStamped.h>




using namespace cv;
using namespace std;
ros::Publisher goal_pub;

double scan_num360;
int half_laser_num



void scan2_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_ERROR("half: %d",half_laser_num);
    cout<<half_laser_num<<endl;
    int int_left_tree, int_right_tree;
    for(int i = 0 ; i < 220;i++){//只在左右各220波束（总共左右各half_laser_num波束）范围内取树
        if(scan->ranges[half_laser_num+i] > 0){
            int_left_tree = i;
            break;
        }
    }

    for(int i = 0 ; i < 220;i++){//只在左右各220波束（总共左右各half_laser_num波束）范围内取树
        if(scan->ranges[half_laser_num-i] > 0){
            int_right_tree = i;
            break;
        }
    }

    cout<<"right:"<<int_right_tree<<endl;

    if(int_left_tree!=0 && int_right_tree!=0){
        geometry_msgs::PointStamped goal;
        goal.header.stamp = scan->header.stamp;
        goal.header.frame_id =scan->header.frame_id;
        goal.point.x = scan->ranges[half_laser_num - int_right_tree] * cos(int_right_tree/scan_num360*M_PI*2.0);
        goal.point.y = (-1.0)* scan->ranges[half_laser_num - int_right_tree] * sin(int_right_tree/scan_num360*M_PI*2.0);
        //goal.point.x = (scan->ranges[405 + int_left_tree] * cos(int_left_tree/886.0*M_PI*2) + scan->ranges[405 - int_right_tree] * cos(int_right_tree/886.0*M_PI*2))/2.0 ;
        //goal.point.y = (scan->ranges[405 + int_left_tree] * sin(int_left_tree/886.0*M_PI*2) - scan->ranges[405 - int_right_tree] * sin(int_right_tree/886.0*M_PI*2))/2.0 ;

        goal.point.z = 0.0 ;


        goal_pub.publish(goal);
    }



}

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh_;
    ros::param::get("~scan_number_360",scan_num360);
    ros::param::get("~half_laser_num",half_laser_num);
    goal_pub = nh_.advertise<geometry_msgs::PointStamped>("/tree_goal",1);
    ros::Subscriber scan_sub = nh_.subscribe("/tree_pt", 1, scan2_callback);
    ros::spin();
    return 0;
}
