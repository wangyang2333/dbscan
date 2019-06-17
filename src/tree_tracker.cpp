//
// Created by xcy on 19-6-11.
//

#include "tree_tracker.h"

/*All right reserved.
 *if tf in rviz can't display, run a roscore first.
 */


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

void tree_tracker::add_to_map(tree new_landmark){
    geometry_msgs::PointStamped tree_point_stamped;
    tree_point_stamped.header.frame_id = "base_laser_link";

    //we'll just use the most recent transform available for our simple example
    tree_point_stamped.header.stamp = ros::Time();

    //just an arbitrary point in space

    tree_point_stamped.point.x = new_landmark.rou * 10.0 * cos((new_landmark.sita*640.0-320.0)/886.0*M_PI*2.0);
    tree_point_stamped.point.y = new_landmark.rou * 10.0 * sin((new_landmark.sita*640.0-320.0)/886.0*M_PI*2.0);
    tree_point_stamped.point.z = 0;

    geometry_msgs::PointStamped base_point;
    listener.transformPoint("odom_combined", tree_point_stamped, base_point);

    geometry_msgs::Point32 pcl_pt;
    pcl_pt.x = base_point.point.x;
    pcl_pt.y = base_point.point.y;
    pcl_pt.z = base_point.point.z;

    map_cloud.points.push_back(pcl_pt);
    map_cloud.channels[0].values.push_back(new_landmark.tree_id);

}

void tree_tracker::remove_from_map(tree landmark_to_remove){
    for(int i= 0; i < map_cloud.channels[0].values.size(); i++){
        if(float(landmark_to_remove.tree_id) == float(map_cloud.channels[0].values[i])){
            map_cloud.channels[0].values.erase(map_cloud.channels[0].values.begin()+i);
            map_cloud.points.erase(map_cloud.points.begin()+i);
            cout<<"remove i"<<endl;
            break;
        }
    }
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
            int bad_index = find_bad_tree(this_trees,this_track );
            add_to_map(this_trees[bad_index]);
            this_track.push_back(this_trees[bad_index]);
        }

    }
    else if(this_trees.size()<this_track.size()){
        int bad_index = find_bad_tree(this_track,this_trees);
        cout<<"remove old tree"<<endl;
        remove_from_map(this_track[bad_index]);
        this_track.erase(this_track.begin()+bad_index);
    }
}

void tree_tracker::tree_callback(const sensor_msgs::LaserScan::ConstPtr& scan){

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

    this_trees.clear();
    for(int i = 0; i < scan->ranges.size(); i++)//push trees into vec
    {
        if(scan->ranges[i] != 0){
            this_trees.push_back(tree(i/640.0, scan->ranges[i]/10.0,ros::Time::now().toSec()));
        }
    }

    if(first_track_flag)// handle first track
    {
        first_track_flag = false;
        this_track = this_trees;

        for(int i = 0; i < this_trees.size(); i++)
        {
            add_to_map(this_trees[i]);
        }
        cout<<"first track"<<endl;
        tree_mtx.unlock();
        return;
    }
    else{
        track_tree();
        cout<<"load scan ....."<<endl;
        for(int i = 0; i< this_track.size(); i++){
            tree_followed.ranges[int(this_track[i].sita*640)] = this_track[i].rou*10;
            tree_followed.intensities[int(this_track[i].sita*640)] =this_track[i].tree_id;
        }
        follow_pub.publish(tree_followed);
        tree_followed.ranges.clear();
        tree_followed.intensities.clear();
    }

    landmark_cloud_pub.publish(map_cloud);
    tree_mtx.unlock();
}