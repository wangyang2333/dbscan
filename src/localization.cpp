//
// Created by xcy on 19-6-12.
//
#include "tree_tracker.h"





int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "tree_tracker");
    tree_tracker my_tree_tracker;
    ros::spin();
    return 0;
}

