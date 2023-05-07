#include"visualization.h"


// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           This Help\n"
            << "-pc1         Name of first .ply File (Just name, Expected to be in Data Directory)\n"
            << "-pc2         Name of second .ply File (Just name, Expected to be in Data Directory)\n"
            << "-c           Save Camera Parameters\n"
            << "-icp         ICP Algorithm\n"
            << "-tricp       Trimeed ICP Algorithm\n"
            << "-it          Maximum Number of Iterations [int]\n"
            << "-e           Epsilon for TrICP [double]\n"
            << "-v           Visualization Mode (Not Point Registration)\n"
            << "-conv1       Convergence Criteria 1 MSE [double]\n"
            << "-conv2       Convergence Criteria 2 MSE Change (TrICP) [double]\n"
            << "-xt          Translation Along the X Axis for pc2 [double]\n"
            << "-yt          Translation Along the Y Axis for pc2 [double]\n"
            << "-zt          Translation Along the Z Axis for pc2 [double]\n"    
            << "-xr          Rotation Around the X Axis for pc2 (degrees) [double]\n"
            << "-yr          Rotation Around the Y Axis for pc2 (degrees) [double]\n"
            << "-zr          Rotation Around the Z Axis for pc2 (degrees) [double]\n"  
            << "-mean        Mean for the Gaussian Noise\n"
            << "-std         STD for the Gaussian Noise\n"            
            << "\n\n";
}

pcl::visualization::PCLVisualizer::Ptr simpleVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  // viewer->addCoordinateSystem (1);
  viewer->initCameraParameters ();
  viewer->loadCameraParameters("../config/camera_params.cam");
  return (viewer);
}

void loadPC(std::string pc_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PLYReader Reader;
    Reader.read(pc_file, *cloud);
}

ICP::MatX pcl2Eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  int samples = cloud->size();
  int dim = 3;

  ICP::MatX A(samples, dim);

  for (int i = 0; i < samples; i++)
  {
    /* Copying Values from Pcl to an Eigen Matrix */
    A(i, 0) = cloud->points[i].x;
    A(i, 1) = cloud->points[i].y;
    A(i, 2) = cloud->points[i].z;
  }

  return A;
}

pcl::PointCloud<pcl::PointXYZ> eigen2PC(ICP::MatX A)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;

  for (int i = 0; i < A.rows(); i++)
  {
    /* Copying Values from Eigen to PclMatrix */
    point.x = A(i, 0);
    point.y = A(i, 1);
    point.z = A(i, 2);
    cloud.points.push_back(point);
  }
  
  return cloud;
}

void addGaussianNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mean, double stddev)
{

  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud->points[i].x = cloud->points[i].x + dist(generator);
    cloud->points[i].y = cloud->points[i].y + dist(generator);
    cloud->points[i].z = cloud->points[i].z + dist(generator);
  }
}
