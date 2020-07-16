//
// Created by xcy on 2020/6/14.
//

#include "fpfh_pcl.h"


pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
boost::mutex cloud_mutex;
pcl::visualization::PCLPlotter plotter;
//pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh_omp;
pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
// structure used to pass arguments to the callback function
struct callback_args {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


// callback function
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    plotter.clearPlots();
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    pcl::PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.clear();
    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

    int num = event.getPointIndex();
    plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", num);
    plotter.plot();
}


void FPFHDriver::FPFH(sensor_msgs::PointCloud& PCL){
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 PCL2;
    sensor_msgs::convertPointCloudToPointCloud2(PCL, PCL2);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(PCL2,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*model);


    bool downSampling = false;
    bool display = true;

    if (downSampling) {
        // create the filtering object
        std::cout << "Number of points before downSampling: " << model->points.size() << std::endl;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(model);
        sor.setLeafSize(0.01, 0.01, 0.01);
        sor.filter(*model);
        std::cout << "Number of points after downSampling: " << model->points.size() << std::endl;
    }

    //  Normal estimation
    auto t1 = chrono::steady_clock::now();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(model);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    //ne.setRadiusSearch(0.03);
    ne.compute(*normals);
    auto t2 = chrono::steady_clock::now();
    auto dt = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    cout << "Time cost of Normal estimation: " << dt << endl;

    // fpfh or fpfh_omp
    fpfh_omp.setInputCloud(model);
    fpfh_omp.setInputNormals(normals);
    fpfh_omp.setSearchMethod(tree);
    fpfh_omp.setNumberOfThreads(8);
    fpfh_omp.setRadiusSearch(0.4);
    fpfh_omp.compute(*fpfhs);
//    fpfh.setInputCloud(model);
//    fpfh.setInputNormals(normals);
//    fpfh.setSearchMethod(tree);
//    fpfh.setRadiusSearch(0.05);
//    fpfh.compute(*fpfhs);
    t1 = chrono::steady_clock::now();
    dt = chrono::duration_cast<chrono::duration<double> >(t1 - t2).count();
    cout << "Time cost of FPFH estimation: " << dt << endl;

    pcl::FPFHSignature33 descriptor;
    for (int i=0; i<10; ++i) {
        int index = i + rand() % model->points.size();
        descriptor = fpfhs->points[index];
        std::cout << " -- fpfh for point "<< index << ":\n" << descriptor << std::endl;
    }

    if (display) {
        plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh",100);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(model, "model");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");
        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(model, normals, 1, 0.1, "normals");  // display every 1 points, and the scale of the arrow is 10
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        // Add point picking callback to viewer:
        struct callback_args cb_args;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
        cb_args.clicked_points_3d = clicked_points_3d;
        cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
        viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
        std::cout << "Shift + click on three floor points, then press 'Q'..." << std::endl;

//        viewer->spin();
//        cloud_mutex.unlock();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100); // Spin until 'Q' is pressed
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}