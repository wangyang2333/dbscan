//
// Created by xcy on 2020/5/30.
//

#include "PCAfilter.h"

void PCA_Eigen(sensor_msgs::PointCloud& PCL){
    cout<<"Begin global PCA"<<endl;
    double xmean, ymean, zmean;
    for (auto pts : PCL.points)
    {
        xmean += pts.x;
        ymean += pts.y;
        zmean += pts.z;
    }
    xmean = xmean / PCL.points.size();
    ymean = ymean / PCL.points.size();
    zmean = zmean / PCL.points.size();
    Eigen::Matrix<double, -1, -1> PCA_X (PCL.points.size(),3);
    //Insert point into the Matrix
    for(int i=0; i<PCL.points.size(); i++){
        //in iteration the i < PCL.point.seze()
        PCA_X(i,0) = PCL.points[i].x - xmean;
        PCA_X(i,1) = PCL.points[i].y - ymean;
        PCA_X(i,2) = PCL.points[i].z - zmean;
    }
    Eigen::Matrix<double, 3, 3> PCA_XXT;
    PCA_XXT = PCA_X.transpose() * PCA_X;
    //Eigen::Vector3d PCA_Eigenvalue = PCA_XXT.eigenvalues();
    Eigen::EigenSolver<Eigen::Matrix3d> es(PCA_XXT);
    //Use Dynamic and Complex matrix, Beacuse the eigenvalue can be complex.
    Eigen::MatrixXcd Lamda = es.eigenvalues();
    Eigen::MatrixXcd U = es.eigenvectors();
    //The principle vector are the columns of Ur
    cout << "The original Matrix" << endl << PCA_XXT <<endl;
    cout << "The eigenvalues Lamdas are:" << endl << Lamda << endl;
    cout << "The eigenvectors matrix U are:" << endl << U << endl;
}

void Voxel_Filter_Hash(sensor_msgs::PointCloud& PCL){
    cout<<"Begin VFH"<<endl;
    double xmean, ymean, zmean;
    for (auto pts : PCL.points)
    {
        xmean += pts.x;
        ymean += pts.y;
        zmean += pts.z;
    }
    xmean = xmean / PCL.points.size();
    ymean = ymean / PCL.points.size();
    zmean = zmean / PCL.points.size();
    Eigen::Matrix<double, -1, -1> VFH_X (PCL.points.size(),3);
    //Insert point into the Matrix
    for(int i=0; i<PCL.points.size(); i++){
        //in iteration the i < PCL.point.seze()
        VFH_X(i,0) = PCL.points[i].x-xmean;
        VFH_X(i,1) = PCL.points[i].y-ymean;
        VFH_X(i,2) = PCL.points[i].z-zmean;
    }
    //Compute Dx Dy Dz
    Eigen::Vector3d Max_Cor = VFH_X.colwise().maxCoeff();
    Eigen::Vector3d Min_Cor = VFH_X.colwise().minCoeff();
    double VG_size = 0.2;
    int Dx, Dy, Dz;// int Dx Dy Dz may cause some problems. Come back later!
    Dx = (Max_Cor(0) - Min_Cor(0)) / VG_size +1;
    Dy = (Max_Cor(1) - Min_Cor(1)) / VG_size +1;
    Dz = (Max_Cor(2) - Min_Cor(2)) / VG_size +1;
    //cout << "Dx: " << Dx << " Dy: " << Dy << " Dz: " << Dz << endl;

    //Compute Voxel Index for Each PTS and Push them into my Hash Table
    int PTS_Num_after_VF = 500;
    std::vector<  pair<std::vector<geometry_msgs::Point32 >,long int> > Hash_table;
    sensor_msgs::PointCloud PCL_after_VF;
    PCL_after_VF.header.frame_id = "velodyne";
    PCL_after_VF.channels.resize(1);
    PCL_after_VF.channels[0].name = "intensities";
    Hash_table.resize(PTS_Num_after_VF);
    for(int i=0; i<PCL.points.size(); i++){
        int hx = floor((VFH_X(i,0) -Min_Cor(0))/VG_size);
        int hy = floor((VFH_X(i,1) -Min_Cor(1))/VG_size);
        int hz = floor((VFH_X(i,2) -Min_Cor(2))/VG_size);
        //cout << "hx: " << hx << " hy: " << hy << " hz: " << hz << endl;
        //Hash PTS to 100 voxels.
        //Careful this may cause over flow

        geometry_msgs::Point32 current_pts;
        current_pts.x = VFH_X(i,0) + xmean;
        current_pts.y = VFH_X(i,1) + ymean;
        current_pts.z = VFH_X(i,2) + zmean;
        long int h = (hx + hy * Dx + hz * Dx * Dy);
        long int hash_num = h % PTS_Num_after_VF; //0~PTS_Num_VF-1
        if(Hash_table[hash_num].first.empty()){
            Hash_table[hash_num].first.push_back(current_pts);
            Hash_table[hash_num].second = h;
        }
        else{
            if(h == Hash_table[hash_num].second){
                Hash_table[hash_num].first.push_back(current_pts);
            }
            else{
                //Handle Conflict: Compute old mean and push into new PCL
                geometry_msgs::Point32 mean_pts;
                for(auto pts : Hash_table[hash_num].first){
                    mean_pts.x += pts.x;
                    mean_pts.y += pts.y;
                    mean_pts.z += pts.z;
                }
                mean_pts.x = mean_pts.x / Hash_table[hash_num].first.size();
                mean_pts.y = mean_pts.y / Hash_table[hash_num].first.size();
                mean_pts.z = mean_pts.z / Hash_table[hash_num].first.size();
                PCL_after_VF.points.push_back(mean_pts);
                //Clear and pushback new pts
                Hash_table[hash_num].first.clear();
                Hash_table[hash_num].first.push_back(current_pts);
                Hash_table[hash_num].second = h;
            }
        }
    }
    cout<<endl<<"i ama here"<<endl;
    for( auto voxel : Hash_table){
        if(!voxel.first.empty())
        {
            geometry_msgs::Point32 mean_pts2;
            for(auto pts : voxel.first){
                mean_pts2.x += pts.x;
                mean_pts2.y += pts.y;
                mean_pts2.z += pts.z;
            }
            mean_pts2.x = mean_pts2.x / voxel.first.size();
            mean_pts2.y = mean_pts2.y / voxel.first.size();
            mean_pts2.z = mean_pts2.z / voxel.first.size();
            PCL_after_VF.points.push_back(mean_pts2);
        }
        else{
            cout<<endl<<"Empty Hash Bucket!"<<endl;
        }
    }
    //tree_visual_cloud_pub.publish(PCL_after_VF);
    //cout << endl << VFH_X << endl;
}