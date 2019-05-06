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
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>




using namespace cv;
using namespace std;

typedef std::queue<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::ImageConstPtr>> mymeasurements;
typedef std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::ImageConstPtr> mymeasurement;
ros::Publisher cloud_pub;
std::mutex measurements_mutex;
std::condition_variable cond;

std::queue<sensor_msgs::ImuConstPtr> imu_queue;
std::queue<sensor_msgs::ImageConstPtr> image_queue;


class featurept
{
public:
    cv::KeyPoint keypoint;
    std::vector<cv::KeyPoint> dangerpoints;
    std::vector<cv::KeyPoint> nextdangerpoints;
    bool indanger = 0;
    int dangernum = 0;
    int popcount = 0;
};

class mymap{
public:
    vector<cv::Mat> keyframes;
    vector<pair<vector<KeyPoint>, vector<Point3f>>> keypointss;
    //void insertkeyframe(cv::Mat kf);
    //void insertkeypoint(std::pair<cv::KeyPoint, Point3d> kp);
};

mymap MAP1;


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    measurements_mutex.lock();
    imu_queue.push(imu_msg);
    measurements_mutex.unlock();
    //cond.notify_one();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    measurements_mutex.lock();
    image_queue.push(image_msg);
    measurements_mutex.unlock();
    if(image_queue.size()>=2)
    {
        //ROS_INFO("successfully push image");
        cond.notify_one();
    }



}

void getmeasurements(mymeasurements& measurements)
{
    while (true)
    {
        if (imu_queue.empty() || image_queue.empty())
        {
            //ROS_WARN("opps: IMU or Image queue empty!");
            return;
        }
        if (!(imu_queue.back()->header.stamp.toSec() > image_queue.front()->header.stamp.toSec() + 0.0))
        {
            ROS_WARN("wait for imu, only should happen at the beginning");
            return;
        }

        if (!(imu_queue.front()->header.stamp.toSec() < image_queue.front()->header.stamp.toSec() + 0.0))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            image_queue.pop();
            continue;
        }
        sensor_msgs::ImageConstPtr img_msg = image_queue.front();
        image_queue.pop();
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_queue.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + 0.0)
        {
            IMUs.emplace_back(imu_queue.front());
            imu_queue.pop();
        }
        IMUs.emplace_back(imu_queue.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace(IMUs, img_msg);
        //ROS_INFO("haha: pop measurements successfully~");
    }
    return;
}


int computeHammingDistance(int i, int j, Mat descriptors_1)
{
    int result = 0;
    HammingLUT lut;
    result += lut(&descriptors_1.at<uchar>(i,0), &descriptors_1.at<uchar>(j,0),32);
    return result;
}
int computedistance2(const KeyPoint& kp1, const KeyPoint& kp2){
    return (kp1.pt.x - kp2.pt.x)*(kp1.pt.x - kp2.pt.x) + (kp1.pt.y - kp2.pt.y)*(kp1.pt.y - kp2.pt.y);
}

void find_danger_points(const Mat& img_1, std::vector<featurept>& featurepts){
    ROS_DEBUG("------------begin find danger points----------");
    featurepts.clear();
    ROS_DEBUG("------------clear last danger-point vector----------");
    Mat descriptors_1;
    std::vector<KeyPoint> key_points_1;
    featurept temp_featurept;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    ROS_DEBUG("------------creat ORB front-end finished----------");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, key_points_1);
    ROS_DEBUG("------------detect corner finished----------");
    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, key_points_1, descriptors_1 );
    ROS_DEBUG("------------compute BRIEF finished----------");
    if(key_points_1.size()<3)return;
    //-- 第三步:将本帧的所有特征点描述子相互异或和，将异或和低于30的作为危险匹配，（描述子256位）
    int counter_j = 0;
    for(auto kpt : key_points_1)
    {
        counter_j++;
        //initial the temp_featurept
        temp_featurept.keypoint = kpt;
        temp_featurept.dangerpoints.clear();
        temp_featurept.indanger = false;
        temp_featurept.dangernum = 0;
        //find danger feature
        int counter_i = 0;
        for(auto kptarget : key_points_1)
        {
            counter_i++;
            if ( (kpt.pt.x==kptarget.pt.x)&&(kpt.pt.y==kptarget.pt.y) )// see if it is the same k-point.
                continue;
            if ( computeHammingDistance(counter_i,counter_j,descriptors_1)<= 8 && computedistance2(kpt,kptarget) >= 2500 )
            {
                temp_featurept.dangerpoints.push_back(kptarget);
                temp_featurept.indanger = true;
                temp_featurept.dangernum++;
            }
        }
        //cout<<"dangernum="<<temp_featurept.dangernum<<endl;
        featurepts.push_back(temp_featurept);
    }
    /*
    Mat out_image;
    drawKeypoints(img_1,featurepts.at(1).dangerpoints,out_image, Scalar(0,255,0),2);
    circle(out_image, featurepts.at(1).keypoint.pt, 10, Scalar(255, 0, 0),2);
    for(auto ppts : featurepts.at(1).dangerpoints)
        line(out_image, featurepts.at(1).keypoint.pt, ppts.pt, Scalar(0, 0, 255), 2);
    cv::imshow("opps", out_image);
    cv::waitKey(3);
    */
}

Mat rosImageToCvMat(sensor_msgs::ImageConstPtr image){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    Mat img_1;
    static float KKK[3][3] = {354.83758544921875000,0,   328.50021362304687500,
                            0, 354.83068847656250000,  240.57057189941406250,
                            0,                     0,        1};
    static float KK[4] = {-0.29249572753906250, 0.07487106323242188, -0.00019836425781250, -0.00031661987304688};
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1,KKK);
    cv::Mat distortion_coefficients = cv::Mat(4,1,CV_32FC1,KK);
    undistort(cv_ptr->image, img_1, camera_matrix, distortion_coefficients);
    return img_1;
}

void track_danger_points(const Mat& temp_mat, std::vector<featurept>& featurepts, const Mat& next_mat)
{
    //for every key-points in the frame.
    //ROS_INFO("-------for every key-points----------");
    vector<cv::Point2f> temp_pts;
    vector<cv::Point2f> next_pts;
    vector<uchar> status;
    vector<float> err;
    for(auto fpts = featurepts.begin(); fpts < featurepts.end(); ++fpts ) {
        //ROS_INFO("-------for every danger-points-----");
        for(auto dangerpts : fpts->dangerpoints) {
            temp_pts.push_back(dangerpts.pt);
            fpts->popcount++;
        }
    }
    if(temp_pts.size()!=0)
        cv::calcOpticalFlowPyrLK(temp_mat, next_mat, temp_pts, next_pts, status, err, cv::Size(21, 21), 3);

    for(auto fpts = featurepts.rbegin(); fpts < featurepts.rend(); ++fpts ) {
        for(auto nextdangerpts = fpts->nextdangerpoints.rbegin(); nextdangerpts != fpts->nextdangerpoints.rend(); ++fpts){
            nextdangerpts->pt = next_pts.back();
            next_pts.pop_back();
        }
    }

    /*TODO
    Mat out_image = temp_mat;
    for(auto fpts : featurepts){
        drawKeypoints(out_image, fpts.dangerpoints, out_image, Scalar(0,255,0),2);//draw danger points
        if(fpts.dangerpoints.size()!=0){
            circle(out_image, fpts.keypoint.pt, 10, Scalar(255, 0, 0),2);//draw key points
            for(auto ppts : fpts.dangerpoints)
                line(out_image, fpts.keypoint.pt, ppts.pt, Scalar(0, 0, 255), 2);//link danger - key points
        }
    }
    int i = 0;
    for(auto tpt : temp_pts)
    {
        if(status[i]=1)line(out_image, tpt, next_pts[i], Scalar(0, 255, 255), 3);// draw optical flow
        i++;
    }

    cv::imshow("track_optical", out_image);
    cv::waitKey(3);
     */



    // cv::imshow("track_optical", temp_mat);
    //ROS_INFO("Calculates an optical flow for danger points");
    /*
    if (temp_pts.size() > 0){
        cv::calcOpticalFlowPyrLK(temp_mat, next_mat, temp_pts, next_pts, status, err, cv::Size(21, 21), 3);

        Mat out_image;
        drawKeypoints(temp_mat, fpts->dangerpoints, out_image, Scalar(0,255,0),2);
        circle(out_image, fpts->keypoint.pt, 10, Scalar(255, 0, 0),2);
        for(auto ppts : fpts->dangerpoints)
            line(out_image, fpts->keypoint.pt, ppts.pt, Scalar(0, 0, 255), 2);
        int i = 0;
        for(auto tpt : temp_pts)
        {
            if(status[i]=1)line(out_image, tpt, next_pts[i], Scalar(0, 255, 255), 4);
            i++;
        }
        cv::imshow("track_optical", out_image);
        cv::waitKey(3);

    }
    */
    //ROS_INFO("Finish calculating an optical flow for danger points");
    //Calculates an optical flow for a sparse feature set using the iterative Lucas-Kanade method with pyramids.

}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}
Mat K = ( Mat_<double> ( 3,3 ) <<354.9553, 0, 327.9541, 0, 355.4596, 242.4097, 0, 0, 1 );
Mat Kfloat = ( Mat_<float> ( 3,3 ) <<354.9553, 0, 327.9541, 0, 355.4596, 242.4097, 0, 0, 1 );
Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
            (
                    ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
                    ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
            );
}



void triangulation (
        const vector< KeyPoint >& keypoints_1,
        const vector< KeyPoint >& keypoints_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        Mat& triangulatedPoints )
{
    Mat T1 = (Mat_<float> (3,4) <<
                                1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
                                R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( keypoints_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoints_2[m.trainIdx].pt, K) );
    }


    cv::triangulatePoints( T1, T2, pts_1, pts_2, triangulatedPoints );
}

Mat mask;
void pose_estimation_2d2d (
        const std::vector<KeyPoint>& keypoints_1,
        const std::vector<KeyPoint>& keypoints_2,
        const std::vector< DMatch >& matches,
        Mat& R, Mat& t )
{
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    Point2d principal_point ( 327.9541, 242.4097 );				//相机主点, TUM dataset标定值
    int focal_length = 355;						//相机焦距, TUM dataset标定值
    Mat essential_matrix, triangulatedPoints2;

    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point,RANSAC, 0.999, 1.0, mask );

    recoverPose( essential_matrix, points1, points2,
                 K, R, t, 200.0, mask,
                 triangulatedPoints2);
}



Mat temp_mat, next_mat;
Eigen::Matrix<double, 4, 4> matrix_T;
bool initialized = false;
void processmeasurements(mymeasurements &measurements)
{
    if(measurements.size()<2)
    {
        ROS_ERROR("measurement is in small size!");
        ROS_ERROR("%d",int(measurements.size()));
        return;
    }
    for(; measurements.size() > 1;)
    {
        temp_mat = next_mat;
        next_mat = rosImageToCvMat(measurements.front().second);
        measurements.pop();
        if(temp_mat.empty())return;
        std::vector<featurept> featurepts;
        //TODO find_danger_points(temp_mat, featurepts);
        //TODO track_danger_points(temp_mat, featurepts, next_mat);

        //--------------must do matches--------------------------//
        vector<KeyPoint> keypoints_1, keypoints_2;
        vector<DMatch> matches, matches2;
        find_feature_matches ( next_mat, temp_mat, keypoints_1, keypoints_2, matches );


        Mat deltaR,deltat;
        if(initialized== false){// if not initialized, do 2d-2d match and triangulation!
            pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, deltaR, deltat );
            Mat triangulatedPoints = Mat_<float>();
            triangulation( keypoints_1, keypoints_2, matches, deltaR, deltat, triangulatedPoints );
            vector<Point3f> goodlandmark;   //this is good 3d points in next frame?
            vector<KeyPoint> goodkeypoint; // this is good 2d points in next frame?
            cout<<mask;
            for( int i=0; i < triangulatedPoints.cols; i++) {
                if (mask.at<int>(i, 0) == 1) {
                    Mat x = triangulatedPoints.col(i);
                    x /= x.at<float>(3.0);
                    Point3f temp3d(x.at<float>(0,0), x.at<float>(1,0), x.at<float>(2,0) );
                    goodlandmark.push_back(temp3d);
                    goodkeypoint.push_back(keypoints_1[i]);//key points 1 is in next frame;
                }
            }
            MAP1.keypointss.emplace_back(goodkeypoint, goodlandmark);
            MAP1.keyframes.push_back(next_mat);

            sensor_msgs::PointCloud cloud;
            cloud.header.frame_id = "robot_odom";
            cloud.points.resize(goodlandmark.size());
            cloud.channels.resize(1);
            cloud.channels[0].name = "intensities";
            cloud.channels[0].values.resize(goodlandmark.size());

            int count = 0;
            for(auto gdlmk : goodlandmark)
            {
                cloud.points[count].x = gdlmk.x;
                cloud.points[count].y = gdlmk.y;
                cloud.points[count].z = gdlmk.z;
                cloud.channels[0].values[count] = count;
                count++;
            }
            cloud_pub.publish(cloud);
            initialized = true;
            //ROS_INFO("begin5");

        }
        else{//if initialized do PnP-RANSAC and triangulation
            vector<Point3f> pts_3d;
            vector<Point2f> pts_2d;
            //ROS_INFO("begin1");
            find_feature_matches( next_mat, MAP1.keyframes.back(), keypoints_1, MAP1.keypointss.back().first, matches2 );
            //ROS_INFO("begin2");
            for(DMatch m:matches2){
                pts_3d.push_back ( MAP1.keypointss.back().second[m.trainIdx]);
                pts_2d.push_back ( keypoints_2[m.queryIdx].pt );
            }
            cout<<"pts3d:"<<pts_3d<<endl;
            cout<<"pts2d:"<<pts_2d<<endl;
            //ROS_INFO("begin3");
            Mat r;
            solvePnP( pts_3d, pts_2d, K, Mat(), r, deltat, false);
            cout<<"t:"<<deltat<<endl;
            //ROS_INFO("begin4");
            cv::Rodrigues ( r, deltaR );
            cout<<"deltaR: "<<deltaR<<endl;
        }
        //--------------tf display------------------------------------------//
        Mat out_image = temp_mat;
        drawKeypoints(out_image, keypoints_1, out_image, Scalar(0,255,0),2);//draw danger points
        cv::imshow("track_optical", out_image);
        cv::waitKey(3);

        Eigen::Matrix<double, 4, 4> matrix_deltaT;
        matrix_deltaT.setIdentity();
        Eigen::Matrix<double, 3, 3> matrix_deltaR;
        Eigen::Matrix<double, 3, 1> matrix_deltat;

        cv2eigen(deltaR,matrix_deltaR );
        cv2eigen(deltat,matrix_deltat);//change cv to eigen matrix
        matrix_deltaT.block<3,3>(0,0) = matrix_deltaR;//fill the delta R t into deltaT.
        matrix_deltaT.block<3,1>(0,3) = matrix_deltat;

        matrix_T = matrix_T * matrix_deltaT;
        cout<<matrix_T<<endl;

        Eigen::Matrix3d RR = matrix_T.block(0,0,3,3);
        Eigen::Quaterniond q(RR);
        tf::Quaternion tf_q;
        tf::quaternionEigenToTF(q,tf_q);

        tf::Vector3 tf_t;
        tf::vectorEigenToTF(matrix_T.block(0,0,3,1),tf_t);


        static tf::TransformBroadcaster br;


        tf::Transform transform2;


        tf::Vector3 vec_fix(1,1,1);
        transform2.setOrigin(vec_fix);


        Eigen::Matrix3d R_fix;
        R_fix << 0, 0, -1,
                1, 0, 0,
                0, -1, 0;
        Eigen::Quaterniond q_fix(R_fix);
        tf::Quaternion tf_qfix;
        tf::quaternionEigenToTF(q_fix,tf_qfix);
        transform2.setRotation(tf_qfix);

        br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "robot_fixed"));



        tf::Transform transform;
        tf::Vector3 vec_zero(0,0,0);
        transform.setOrigin(tf_t);
        //transform.setOrigin( tf_t );
        transform.setRotation(tf_q);



        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_fixed", "robot_odom"));
        //vector<Point3d> points;
        //TODO triangulation( keypoints_1, keypoints_2, matches, R, t, points );
    }
    //ROS_INFO("finish track danger points");
}


mymeasurements measurements;
void process(){
    while(true)
    {
        std::unique_lock<std::mutex> locker(measurements_mutex);
        cond.wait(locker);
        getmeasurements(measurements);
        processmeasurements(measurements);
        locker.unlock();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub = nh_.subscribe("/mynteye/imu/data_raw", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber image_sub = nh_.subscribe("/mynteye/left/image_color", 2000, image_callback);
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 50);
    matrix_T.setIdentity();
    std::thread measurement_process{process};
    //optimization();
    ros::spin();
    return 0;
}