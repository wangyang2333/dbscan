//
// Created by xcy on 2020/5/30.
//

#ifndef SRC_GROUNDREMOVALRANSAC_H
#define SRC_GROUNDREMOVALRANSAC_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace std;

struct planeResidual {
    planeResidual(double xi, double yi, double zi)
            : xi_(xi),yi_(yi),zi_(zi) {}
    template <typename T> bool operator()(const T* const a,
                                          const T* const b,
                                          const T* const c,
                                          T* residual) const {
        residual[0] = a[0] * xi_ + b[0] * yi_ + c[0] * zi_ - 1.0;
        return true;
    }
private:
    const double xi_;
    const double yi_;
    const double zi_;
};


class ransacDriver {
private:
    double groundZMax = -0.0;
    double groundZMin = -2.0;

    double inlierRatio = 0.5;
    int sampleNum = 5;
    double confidence = 0.99;

    double inlinerThreshold = 0.05;

    double ratioCondition = 0.8;

    double upperBorder = 0.15;
public:
    void groundRemove(sensor_msgs::PointCloud& PCLin);
    void groundRemoveSided(sensor_msgs::PointCloud &PCLin);
    sensor_msgs::PointCloud PCLforOutput;


    void setGroundZMaxAndMin(double gzmax, double gzmin){groundZMax = gzmax; groundZMin = gzmin;}
    void setInlinerRatio(double iratio){inlierRatio = iratio;}
    void setSampleNum(int num){sampleNum = num;}
    void setConfidence(double cfdc){confidence = cfdc;}
    void setInlinerThreshold(double inthres){inlinerThreshold = inthres;}
    void setRatioCondition(double rc){ ratioCondition = rc;}
    void setUpperBorder(double ub){upperBorder = ub;}

    enum {CANDIDATE = 0, INLINER = 1};
};


#endif //SRC_GROUNDREMOVALRANSAC_H
