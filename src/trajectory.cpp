#include <iostream>
#include"visualization.h"
#include"scanMatching.h"
#include <pcl/registration/icp.h>

using namespace std::chrono_literals;

int countDigit(long long n)
{
    if (n == 0)
    return 1;
    int count = 0;
    while (n != 0) {
        n = n / 10;
        ++count;
        }
    return count;
}

std::string get_filename(int i)
{
    int total_digits = 10;
    int zeros = total_digits - countDigit(i);
    std::string pcdname = "";
    for (int j = 0; j < zeros; j++)
    {
        pcdname += "0";
    } 

    pcdname += std::to_string(i);

    pcdname = "../kitti_pcd2/" + pcdname + ".pcd";

    return pcdname;
}

int main ()
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    

    int total_number_pcd = 300 /*960*/;

    std::string filename;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    viewer = simpleVis();

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Setting up the Pointcloud Data Structures

    /* PCL Pointclouds*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result (new pcl::PointCloud<pcl::PointXYZ>);

    /* Eigen Matrix Pointclouds*/
    ICP::MatX src_cloud;
    ICP::MatX dst_cloud;

    ICP::MatX result_mat;

    /*ICP Results*/
    ICP::Mat3 R; // Resulting Rotation Matrix
    ICP::Vec3 t; // Resulting Translation Vector
    ICP::Mat4 T; // Resulting Transformation Matrix

    // Reading the First Point Cloud
    filename = get_filename(0);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../kitti_pcd2/0000000000.pcd", *cloud_src) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_color(cloud_src, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZ> (cloud_src, original_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    for (int i =1; i < total_number_pcd; i++)
    {
        // Loading the Consecutive Pointclouds
        filename = get_filename(i);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_dst) == -1) //* load the file
        {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
        }
        std::cout << " Number of Points Target: "<< filename << " " << cloud_dst->size() << std::endl; 

        /*Define Pointclouds*/
        src_cloud = pcl2Eigen(cloud_src);

        dst_cloud = pcl2Eigen(cloud_dst);

        /*Initilize scanMatching Object*/
        scanMatching registration(dst_cloud, src_cloud);
        registration.setMaxIterations(100);
        registration.setEpsilon(0.8);
        registration.setMinMSE(0.001);
        registration.setMinChangeMSE(0.0001);

        /*Perform Pointcloud Registration Via ICP*/

        registration.tricp(result_mat, R, t, T);

        // Obtain the transformation that aligned cloud_source to cloud_source_registered
        transform.matrix() = T.cast <float>()*transform.matrix();

        std::cout << transform.matrix() << std::endl;

        pcl::transformPointCloud (*cloud_dst, *cloud_result, transform);

        viewer->addCoordinateSystem(2, transform);

        //viewer->addPointCloud<pcl::PointXYZ> (cloud_result, original_color, filename);

        *cloud_src = *cloud_dst;

    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}