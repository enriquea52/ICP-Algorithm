
#include"visualization.h"
#include"scanMatching.h"

using namespace std::chrono_literals;
using namespace std::chrono;

std::string plyfilename1;
std::string plyfilename2;

bool rgb(true);
bool camParams(false);
bool mode(true);
bool visualize(false);
int maxIterations = 0;
double epsilon = 0.5;
double minMSE = 1e-06;
double minChangeMSE = 1e-06;
double xt = 0.0, yt = 0.0, zt = 0.0;
double xr = 0.0, yr = 0.0, zr = 0.0;
double noise_mean = 0.0, noise_std = 0.000001;
ICP::Afine3 transform = ICP::Afine3::Identity();

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

  // -----------------------------
  // ----- Running Examples ------
  // -----------------------------
  // ./ICP -icp -it 50 -conv1 0.001 -con2 0.00001 -pc1 chopper_corrupted1.ply -pc2 chopper_corrupted2.ply -zr 5 -yt 2
  // ./ICP -mean 0 -std 0.01 -tricp -e 0.3 -it 300 -conv1 0.001 -con2 0.00001 -pc1 sandal_corrupted1.ply -pc2 sandal_corrupted2.ply -zr 20 -xt 0.2
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-pc1") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-pc1", plyfilename1);
    std::cout << "Pointcloud 1 is: " << plyfilename1 << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-pc2") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-pc2", plyfilename2);
    std::cout << "Pointcloud 2 is: " << plyfilename2 << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    camParams = true;
    std::cout << "Save Camera Params Option ON" << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-icp") >= 0)
  {
    mode = true;
    std::cout << "ICP Algorithm Selected" << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-tricp") >= 0)
  {
    mode = false;
    std::cout << "Trimmed ICP Algorithm Selected" << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-it") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-it", maxIterations);
    std::cout << "Maximum Number of Iterations Set to: " << maxIterations << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-e") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-e", epsilon);
    std::cout << "Epsilon for Trimmed ICP set to: " << epsilon << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
    visualize = true;
    std::cout << "Visualization Mode Selected" << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-conv1") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-conv1", minMSE);
    std::cout << "Minimum MSE Set To: " << minMSE << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-conv2") >= 0)
  {
    pcl::console::parse_argument(argc, argv, "-conv2", minChangeMSE);
    std::cout << "Minimum Change in MSE Set To: " << minChangeMSE << std::endl;
  }
  if (pcl::console::find_argument (argc, argv, "-xt") >= 0)
  {
  // Translation Along X Axis
  pcl::console::parse_argument(argc, argv, "-xt", xt);
  }
  if (pcl::console::find_argument (argc, argv, "-yt") >= 0)
  {
  // Translation Along Y Axis
  pcl::console::parse_argument(argc, argv, "-yt", yt);
  }
  if (pcl::console::find_argument (argc, argv, "-zt") >= 0)
  {
  // Translation Along Z Axis
  pcl::console::parse_argument(argc, argv, "-zt", zt);
  }
  if (pcl::console::find_argument (argc, argv, "-xr") >= 0)
  {
  pcl::console::parse_argument(argc, argv, "-xr", xr);
  // Rotation Around X Axis
  transform.rotate (ICP::Axis (xr*pi/180, ICP::Vec3::UnitX()));
  }
  if (pcl::console::find_argument (argc, argv, "-yr") >= 0)
  {
  pcl::console::parse_argument(argc, argv, "-yr", yr);
  // Rotation Around Y Axis
  transform.rotate (ICP::Axis (yr*pi/180, ICP::Vec3::UnitY()));
  }
  if (pcl::console::find_argument (argc, argv, "-zr") >= 0)
  {
  pcl::console::parse_argument(argc, argv, "-zr", zr);
  // Rotation Around Y Axis
  transform.rotate (ICP::Axis (zr*pi/180, ICP::Vec3::UnitZ()));
  }
  if (pcl::console::find_argument (argc, argv, "-mean") >= 0)
  {
  pcl::console::parse_argument(argc, argv, "-mean", noise_mean);
  }
  if (pcl::console::find_argument (argc, argv, "-std") >= 0)
  {
  pcl::console::parse_argument(argc, argv, "-std", noise_std);
  }
  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------

  // Original Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  // Rotated/Translated Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Registered Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
 
  // Define a translation of --- meters on the x axis.
  transform.translation() << xt, yt, zt;
  
  // Loading Original Point Cloud
  loadPC("../data/"+plyfilename1, src_cloud_ptr);

  loadPC("../data/"+plyfilename2, dst_cloud_ptr);

  // Creating a Synthetic Pointlcoud with a pre-defined rotation
  pcl::transformPointCloud (*dst_cloud_ptr, *dst_cloud_ptr, transform);

  // Adding Noise to the Source Point Cloud
  addGaussianNoise(src_cloud_ptr, noise_mean, noise_std);

  // Adding Noise to the Destination Point Cloud
  addGaussianNoise(dst_cloud_ptr, noise_mean, noise_std);

  std::cout << "Number of Points src Pc: " << src_cloud_ptr->size() << std::endl;

  std::cout << "Number of Points dst Pc: " << dst_cloud_ptr->size() << std::endl;

  //---------------------------------------------
  // -----Visualizing Point Clouds and Setup-----
  //---------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer_init;
  pcl::visualization::PCLVisualizer::Ptr viewer_result;

  if (rgb)
  {
    viewer_init = simpleVis();
    viewer_result = simpleVis();
  }

  viewer_init->setWindowName("Initial Conditions");
  viewer_result->setWindowName("Alignment Result");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(src_cloud_ptr, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dst_color(dst_cloud_ptr, 255, 0, 0);

  // Displaying Original Pointclouds 


  viewer_init->addPointCloud<pcl::PointXYZ> (src_cloud_ptr, src_color, "src cloud");
  viewer_init->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "src cloud");

  if(!visualize){
  viewer_init->addPointCloud<pcl::PointXYZ> (dst_cloud_ptr, dst_color, "dst cloud");
  viewer_init->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "dst cloud");
  }

  if(!visualize)
  {
    /*Define Pointclouds*/
    ICP::MatX src_cloud = pcl2Eigen(src_cloud_ptr);

    ICP::MatX dst_cloud = pcl2Eigen(dst_cloud_ptr);

    ICP::MatX result_mat;

    /*ICP Results*/
    ICP::Mat3 R; // Resulting Rotation Matrix
    ICP::Vec3 t; // Resulting Translation Vector
    ICP::Mat4 T; // Resulting Transformation Matrix

    /*Initilize scanMatching Object*/
    scanMatching registration(dst_cloud, src_cloud);
    registration.setMaxIterations(maxIterations);
    registration.setEpsilon(epsilon);
    registration.setMinMSE(minMSE);
    registration.setMinChangeMSE(minChangeMSE);

    /*Perform Pointcloud Registration Via ICP*/
    auto start = high_resolution_clock::now();

    if (mode)
    {registration.icp(result_mat, R, t, T);}
    else
    {registration.tricp(result_mat, R, t, T);}
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "Computation Time: " << duration.count()/1000000.0 << " Seconds" << std::endl;

    /*Convert Resulting Pointcloud Back to PCL Pointcloud for Visualization*/
    *registered_ptr = eigen2PC(result_mat);

    std::cout << "R: \n" << R << std::endl
              << "t: \n" << t << std::endl
              << "T: \n" << T << std::endl
              << "Original Transform: \n" << transform.matrix() << std::endl;

    /*Display Metrics*/
    registration.rotationalError(R, transform.matrix().block(0, 0, 3, 3));
    registration.translationError(T.block(0, 3, 3, 1), transform.matrix().block(0, 3, 3, 1));

    // Displaying Results
    viewer_result->addPointCloud<pcl::PointXYZ> (registered_ptr, src_color, "registered cloud");
    viewer_result->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "registered cloud");

    viewer_result->addPointCloud<pcl::PointXYZ> (dst_cloud_ptr, dst_color, "dst cloud");
    viewer_result->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "dst cloud");
  }
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer_result->wasStopped ())
  {
    viewer_init->spinOnce(100);
    viewer_result->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  if (camParams){viewer_result->saveCameraParameters("../config/camera_params.cam");}

}