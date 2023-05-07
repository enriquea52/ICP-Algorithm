#pragma once
#include "generic.h"
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

// --------------
// -----Help-----
// --------------
void printUsage (const char* progName);

pcl::visualization::PCLVisualizer::Ptr simpleVis ();

void loadPC(std::string pc_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void addGaussianNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mean, double stddev);

// Pointcloud Conversion Functions
ICP::MatX pcl2Eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZ> eigen2PC(ICP::MatX A);