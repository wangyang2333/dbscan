//
// Created by xcy on 19-6-11.
//

#include "tree_tracker.h"
/*All right reserved.
 *if tf in rviz can't display, run a roscore first.
 */

using namespace cv;
using namespace std;

class tree{
public:
    double sita;
    double rou;
    tree(double x_,double y_){
        sita = x_;
        rou = y_;
    }
};

class tree_traker{
public:
    std::vector<tree> last_trees;
    vector<tree> this_track;
    std::vector<tree> this_trees;
    void track_tree();
    bool first_track_flag;

    tree_traker(){
        first_track_flag = true;
    }
};

void tree_traker::track_tree(){

}




tree_traker my_tree_tracker;
void tree_callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    my_tree_tracker.last_trees = my_tree_tracker.this_trees;

    for(int i = 0; i < scan->ranges.size(); i++)//push trees into vec
    {
        if(scan->ranges[i] != 0){
            my_tree_tracker.this_trees.push_back(tree(i/640.0, scan->ranges[i]/10.0));
        }
    }

    if(my_tree_tracker.first_track_flag) return;// handle first track

    //-------------------------track------------------------------------------
    my_tree_tracker.track_tree();

}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "tree_tracker");
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub = nh_.subscribe("/tree_pt", 1, tree_callback);
    ros::spin();
    return 0;
}
