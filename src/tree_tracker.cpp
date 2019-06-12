//
// Created by xcy on 19-6-11.
//

#include "tree_tracker.h"

/*All right reserved.
 *if tf in rviz can't display, run a roscore first.
 */

std::mutex tree_mtx;


tree_tracker my_tree_tracker;

sensor_msgs::LaserScan tree_followed;
ros::Publisher follow_pub; 

void tree_callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    //my_tree_tracker.last_trees = my_tree_tracker.this_trees;

    tree_mtx.lock();

    tree_followed.header.stamp=scan->header.stamp;
    tree_followed.header.frame_id=scan->header.frame_id;
    tree_followed.angle_min=scan->angle_min;
    tree_followed.angle_max=scan->angle_max;
    tree_followed.angle_increment=scan->angle_increment;
    tree_followed.time_increment=scan->time_increment;
    tree_followed.range_min=scan->range_min;
    tree_followed.range_max=scan->range_max;
    tree_followed.ranges.resize(1000);
    tree_followed.intensities.resize(1000);

    my_tree_tracker.this_trees.clear();
    for(int i = 0; i < scan->ranges.size(); i++)//push trees into vec
    {
        if(scan->ranges[i] != 0){
            my_tree_tracker.this_trees.push_back(tree(i/640.0, scan->ranges[i]/10.0,ros::Time::now().toSec()));
        }
    }

    if(my_tree_tracker.first_track_flag)// handle first track
    {
        my_tree_tracker.first_track_flag = false;
        my_tree_tracker.this_track = my_tree_tracker.this_trees;
        cout<<"first track"<<endl;
        tree_mtx.unlock();
        return;
    }
    else{
        my_tree_tracker.track_tree();
        cout<<"load scan ....."<<endl;
        for(int i = 0; i< my_tree_tracker.this_track.size(); i++){
            tree_followed.ranges[int(my_tree_tracker.this_track[i].sita*640)] = my_tree_tracker.this_track[i].rou*10;
            tree_followed.intensities[int(my_tree_tracker.this_track[i].sita*640)] =my_tree_tracker.this_track[i].tree_id;
        }
        follow_pub.publish(tree_followed);
        tree_followed.ranges.clear();
        tree_followed.intensities.clear();
    }
    tree_mtx.unlock();

    //-------------------------track------------------------------------------


}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "tree_tracker");
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub = nh_.subscribe("/tree_pt", 1, tree_callback);
    follow_pub = nh_.advertise<sensor_msgs::LaserScan>("/tree_followed",1);



    dlib::matrix<long int> cost(3,3);
    cost =  7,5, 3,
            9, 2,1,
            3,4, 5;
    cost(1,0) = 8;

    std::vector<long> assignment = max_cost_assignment(cost);

    // This prints optimal assignments:  [2, 0, 1] which indicates that we should assign
    // the person from the first row of the cost matrix to job 2, the middle row person to
    // job 0, and the bottom row person to job 1.
    for (unsigned int i = 0; i < assignment.size(); i++)
        cout << assignment[i] << std::endl;

    // This prints optimal cost:  16.0
    // which is correct since our optimal assignment is 6+5+5.
    cout << "optimal cost: " << assignment_cost(cost, assignment) << endl;

    ros::spin();
    return 0;
}
