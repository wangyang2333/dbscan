//
// Created by xcy on 2020/8/10.
//

#include "treeCenterLocalization.h"
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "tree_tracker");

    TreeCenterLocalization localizationDriver;
    ros::spin();
    return 0;
}

