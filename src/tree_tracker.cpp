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

void tree_tracker::change_frame(tree tree_in, string frame_in, geometry_msgs::Point32& pt_out, string frame_out){
    geometry_msgs::PointStamped tree_point_stamped;
    tree_point_stamped.header.frame_id = frame_in;
    tree_point_stamped.header.stamp = ros::Time();

    tree_point_stamped.point.x = tree_in.rou * 10.0 * cos((tree_in.sita*640.0-320.0)/886.0*M_PI*2.0);
    tree_point_stamped.point.y = tree_in.rou * 10.0 * sin((tree_in.sita*640.0-320.0)/886.0*M_PI*2.0);
    tree_point_stamped.point.z = 0;

    geometry_msgs::PointStamped base_point;
    listener.transformPoint(frame_out, tree_point_stamped, base_point);

    geometry_msgs::Point32 pcl_pt;
    pt_out.x = base_point.point.x;
    pt_out.y = base_point.point.y;
    pt_out.z = base_point.point.z;
}


void tree_tracker::add_to_map(tree new_landmark){
    //add new tree to map_cloud
    geometry_msgs::Point32 pcl_pt;
    change_frame(new_landmark,"base_laser_link",pcl_pt,"base_link");
    geometry_msgs::Point32 final_pt;
    change_frame(pcl_pt,"case_link",final_pt,"odom_combined");

    map_cloud.points.push_back(final_pt);
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

void tree_tracker::change_frame(geometry_msgs::Point32 pt_in, string frame_in, geometry_msgs::Point32& pt_out, string frame_out){
    geometry_msgs::PointStamped pt_stamped_in;
    pt_stamped_in.header.frame_id = frame_in;
    pt_stamped_in.header.stamp = ros::Time();

    pt_stamped_in.point.x = pt_in.x;
    pt_stamped_in.point.y = pt_in.y;
    pt_stamped_in.point.z = pt_in.z;

    geometry_msgs::PointStamped pt_stamped_out;
    listener.transformPoint(frame_out, pt_stamped_in, pt_stamped_out);

    pt_out.x = pt_stamped_out.point.x;
    pt_out.y = pt_stamped_out.point.y;
    pt_out.z = pt_stamped_out.point.z;
}




double tree_tracker::pts32_error(geometry_msgs::Point32 pts1, geometry_msgs::Point32 pts2){
    return (pts1.x-pts2.x)*(pts1.x-pts2.x) + (pts1.y-pts2.y)*(pts1.y-pts2.y);
}


struct tree_Residual{
    tree_Residual(geometry_msgs::Point32 pts_map, geometry_msgs::Point32 pts_track)
            : pts_map_(pts_map),pts_track_(pts_track) {}
    template <typename T> bool operator()(const T* const x,
                        const T* const y,
                        const T* const sita,
                        T* residual) const {


        double x_map, y_map;
        double x_track, y_track;
        x_map = pts_map_.x;
        y_map = pts_map_.y;

        x_track = pts_track_.x;
        y_track = pts_track_.y;



        cout<<"deltax!: "<<x_map*cos(sita[0]) + y_map*sin(sita[0]) - x[0] - x_track<<endl;
        cout<<"deltay!: "<<-x_map*sin(sita[0]) + y_map*cos(sita[0]) - y[0] - y_track<<endl;

        residual[0]=(x_map*cos(sita[0]) + y_map*sin(sita[0]) - x[0] - x_track)*
                (x_map*cos(sita[0]) + y_map*sin(sita[0]) - x[0] - x_track)+
                (-x_map*sin(sita[0]) + y_map*cos(sita[0]) - y[0] - y_track)*
                (-x_map*sin(sita[0]) + y_map*cos(sita[0]) - y[0] - y_track);

        return true;
    }
private:
    const geometry_msgs::Point32 pts_map_;
    const geometry_msgs::Point32 pts_track_;
};

void tree_tracker::localize(){
    ceres::Problem problem;
    double x,y,sita;
    x = double(pr2_pose.pose.position.x);
    y = double(pr2_pose.pose.position.y);
    sita = last_sita;


    for(int i = 0; i < map_cloud.points.size(); i ++){
        geometry_msgs::Point32 temp_map_32;
        temp_map_32 = map_cloud.points[i];
        //change_frame(map_cloud.points[i],"odom_combined",temp_map_32,"base_link");// error! this is changing need to be compute in optimization;
        for(int j = 0; j < this_track.size(); j++){
            geometry_msgs::Point32 temp_track_32;
            cout<<"I am here\n";

            cout<<"id_track:"<<float(this_track[j].tree_id)<<endl;
            cout<<"id_tree:"<<float(map_cloud.channels[0].values[i])<<endl;
            if(float(this_track[j].tree_id) == float(map_cloud.channels[0].values[i])){
                change_frame(this_track[j],"base_laser_link",temp_track_32,"base_link");
                cout<<"map:"<<temp_map_32<<endl;
                cout<<"track:"<<temp_track_32<<endl;
                problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<tree_Residual, 1, 1 , 1, 1>(
                                new tree_Residual(temp_map_32,temp_track_32)),
                        NULL,
                        &x, &y, &sita);
                continue;
            }
        }
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final   x: " << x << " y: " << y << " sita: " << sita <<"\n";
    last_sita = sita;

    pr2_pose.header.frame_id = "odom_combined";
    pr2_pose.header.stamp = ros::Time();
    pr2_pose.pose.position.x = x;
    pr2_pose.pose.position.y = y;
    pr2_pose.pose.position.z = 0;

    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(sita),pr2_pose.pose.orientation);
    pr2_pose_publisher.publish(pr2_pose);

    my_transform.setOrigin(tf::Vector3(x,y,0));
    my_transform.setRotation(tf::createQuaternionFromYaw(sita));
    my_br.sendTransform(tf::StampedTransform(my_transform,ros::Time::now(),"odom_combined","case_link"));

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

    RNG rng;

    this_trees.clear();
    for(int i = 0; i < scan->ranges.size(); i++)//push trees into vec
    {
        if(scan->ranges[i] != 0){
            this_trees.push_back(tree(i/640.0, scan->ranges[i]/10.0,rng.uniform(0.f,1.f)));
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
        cout<<"i am in else\n";
    }
    cout<<"i am in call back! \n";
    if(this_track.size() >= 2){
        localize();
    }
    else{
        ROS_WARN("no enough trees in FOV!");
    }
    landmark_cloud_pub.publish(map_cloud);
    tree_mtx.unlock();
}
