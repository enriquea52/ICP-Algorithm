#pragma once
#include "generic.h"
#include "../nanoflann.hpp"
#include "SO3.hpp"

using PC_type = ICP::MatX;

using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<PC_type, 3, nanoflann::metric_L2_Simple>;

class scanMatching{

  private:

  int max_iterations = 200;
  double N;
  double epsilon = 0.8;
  double minMSE;
  double minChangeMSE;

  ICP::MatX src_cloud; 
  ICP::MatX dst_cloud;
  ICP::MatX correspondances;
  ICP::Vec3 Pmean;
  ICP::Vec3 Qmean;
  ICP::Mat3 K;
  ICP::Mat3 R = ICP::Mat3::Identity(), R_temp;
  ICP::Vec3 t = ICP::Vec3::Zero(), t_temp;
  ICP::Mat4 T = ICP::Mat4::Zero();
  Eigen::JacobiSVD<ICP::Mat3, Eigen:: ComputeFullU | Eigen::ComputeFullV> svd;
  ICP::MatX temp_dst;
  ICP::MatX temp_src;

  public:
  scanMatching(ICP::MatX src_cloud, ICP::MatX dst_cloud);

  void set_clouds(ICP::MatX src_cloud, ICP::MatX dst_cloud);

  void icp(ICP::MatX &result_cloud, ICP::Mat3 &result_R, 
           ICP::Vec3 &result_t, ICP::Mat4 &result_T);

  void tricp(ICP::MatX &result_cloud, ICP::Mat3 &result_R, 
             ICP::Vec3 &result_t, ICP::Mat4 &result_T);

  void pairing(my_kd_tree_t &my_tree, ICP::MatX &correspondances, bool sort = 0, int Npo = 0);

  void motion(ICP::MatX &dst, ICP::MatX &src,
              ICP::Mat3 &R_temp, ICP::Vec3 &t_temp, ICP::Mat4 &T );
              
  void rotationalError(SO3::Mat3 R, SO3::Mat3 Q);

  void translationError(SO3::Vec3 t1, SO3::Vec3 t2);

  void setEpsilon(double epsilon);

  void setMaxIterations(int max_iterations);

  void setMinMSE(double minMSE);

  void setMinChangeMSE(double setMinChangeMSE);

};

void eigen_sort_rows_by_head(ICP::MatX& A_nx3, int Npo);
