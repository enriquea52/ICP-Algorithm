#include"scanMatching.h"
#include <algorithm>

#define pi 3.14159265359 

using namespace std;

void eigen_sort_rows_by_head(ICP::MatX& A_nx3, int Npo)
{
    std::vector<ICP::VecX> vec;
    #pragma omp parallel for num_threads(8)
    for (int64_t i = 0; i < A_nx3.rows(); ++i)
        vec.push_back(A_nx3.row(i));

    // std::sort(vec.begin(), vec.end(), [](ICP::VecX const& t1, ICP::VecX const& t2){ return t1(0) < t2(0); } );
    std::nth_element(vec.begin(),vec.begin() + Npo, vec.end(), [](ICP::VecX const& t1, ICP::VecX const& t2){ return t1(0) < t2(0); } );

    #pragma omp parallel for num_threads(8)
    for (int64_t i = 0; i < A_nx3.rows(); ++i)
        A_nx3.row(i) = vec[i];
};

scanMatching::scanMatching(ICP::MatX dst_cloud, ICP::MatX src_cloud): src_cloud(src_cloud), dst_cloud(dst_cloud)
{
  /* src_cloud: Pointcloud to be aligned  defined as a ICP::MatX of size Nx3 */
  /* dst_cloud: Desired Pointcloud to Result from the Point Registration Process defined as a ICP::MatX of size Nx3 */

  T.block(0, 0, 3, 3) = ICP::Mat3::Identity();
  N = src_cloud.rows();
  correspondances = ICP::MatX(src_cloud.rows(), 3);
}

void scanMatching::set_clouds(ICP::MatX src_cloud, ICP::MatX dst_cloud)
{
  /* src_cloud: Pointcloud to be aligned  defined as a ICP::MatX of size Nx3 */
  /* dst_cloud: Desired Pointcloud to Result from the Point Registration Process defined as a ICP::MatX of size Nx3 */
  this->src_cloud = src_cloud;
  this->dst_cloud = dst_cloud;
  R = ICP::Mat3::Identity();
  t = ICP::Vec3::Zero();
  T = ICP::Mat4::Zero();
  T.block(0, 0, 3, 3) = ICP::Mat3::Identity();
  N = src_cloud.rows();
  correspondances = ICP::MatX(src_cloud.rows(), 3);
}

void scanMatching::setEpsilon(double epsilon)
{
  /* Setting Epsilon Value (Percentage of Original Data) for trICP algorithm*/
  this->epsilon = epsilon;
}
void scanMatching::setMaxIterations(int max_iterations)
{
  /* Setting Epsilon Value (Percentage of Original Data) for trICP algorithm*/
  this->max_iterations = max_iterations;
}

void scanMatching::setMinMSE(double minMSE)
{
  /* Setting Minimum MSE Error for Convergence Test*/
  this->minMSE = minMSE;
}

void scanMatching::setMinChangeMSE(double minChangeMSE)
{
    /* Setting Minimum Change in MSE Error for Convergence Test*/
    this->minChangeMSE = minChangeMSE;
}


void scanMatching::icp(ICP::MatX &result_cloud, ICP::Mat3 &result_R, 
                       ICP::Vec3 &result_t, ICP::Mat4 &result_T)
{
  my_kd_tree_t my_tree(3, std::cref(dst_cloud));
  my_tree.index->buildIndex();

  double prevMSE = 1000000;
  double MSE = 0;
  double dMSE = 0;

  for (int it = 0; it < max_iterations; it++)
  {
    /* Finding Correspondances of src_cloud to ground truth cloud 1 */

    pairing(my_tree, correspondances);

    temp_dst = dst_cloud(correspondances.col(2), Eigen::placeholders::all);

    MSE = correspondances.col(0).sum();

    dMSE = std::abs(MSE - prevMSE);

    /*Check for Convergence*/
    if (MSE < minMSE || dMSE < minChangeMSE)
    {
      std::cout << "Optimal Solution Found in Iteration: " << it << std::endl;
      std::cout << "MSE: " << MSE << " minChangeMSE: " << dMSE << std::endl;
      result_cloud = src_cloud; result_R = R; result_t = t; 
      result_T = T;
      result_T.block(0, 0, 3, 3) = R;
      result_T.block(0, 3, 3, 1) = t;
      result_T.block(3, 0, 1, 3).setZero();
      result_T(3, 3) = 1;
      return;
    }

    prevMSE = MSE;

    /* Compute motion that minimises mean square error (MSE) between paired points. */

    motion(temp_dst, src_cloud, R_temp, t_temp, T);

    R = R_temp*R;

    t = t + t_temp;

    /*Apply motion to src_cloud*/

    src_cloud = ((R_temp*src_cloud.transpose()).colwise() + t_temp).transpose();

  }
  std::cout << "Reaching Maximum Number of Iterations" << std::endl;
  std::cout << "MSE: " << MSE << std::endl;
  result_cloud = src_cloud; result_R = R; result_t = t;
  result_T.block(0, 0, 3, 3) = R;
  result_T.block(0, 3, 3, 1) = t;
  result_T.block(3, 0, 1, 3).setZero();
  result_T(3, 3) = 1;
  return;
}


void scanMatching::tricp(ICP::MatX &result_cloud, ICP::Mat3 &result_R, 
                         ICP::Vec3 &result_t, ICP::Mat4 &result_T)
{
  my_kd_tree_t my_tree(3, std::cref(dst_cloud));
  my_tree.index->buildIndex();

  double Sts_prev = 1000000;
  double Sts = 0;
  int Npo = N*epsilon;
  double trMSE = 0;
  double dtrMSE = 0;

  ICP::MatX temp_src(Npo,3);

  ICP::MatX temp_dst(Npo,3);

  for (int it = 0; it < max_iterations; it++)
  {
    /* Finding Correspondances of src_cloud to ground truth cloud 1 */
    pairing(my_tree, correspondances, true, Npo);

    temp_src = src_cloud(correspondances.col(1), Eigen::placeholders::all).block(0, 0, Npo,3);

    temp_dst = dst_cloud(correspondances.col(2), Eigen::placeholders::all).block(0, 0, Npo,3);

    Sts = correspondances(Eigen::seqN(0,Npo), 0).sum();

    trMSE = Sts/Npo;

    dtrMSE = abs(Sts - Sts_prev);

    /*Check for Convergence*/
    if (trMSE < minMSE || dtrMSE < minChangeMSE)
    {
      std::cout << "Optimal Solution Found in Iteration: " << it << std::endl;
      std::cout << "MSE: " << trMSE << " minChangeMSE: " << dtrMSE << std::endl;
      result_cloud = src_cloud; result_R = R; result_t = t; 
      result_T = T;
      result_T.block(0, 0, 3, 3) = R;
      result_T.block(0, 3, 3, 1) = t;
      result_T.block(3, 0, 1, 3).setZero();
      result_T(3, 3) = 1;
      return;
    }

    Sts_prev = Sts;

    /* Compute motion that minimises mean square error (MSE) between paired points. */

    motion(temp_dst, temp_src, R_temp, t_temp, T);

    R = R_temp*R;

    t = t + t_temp;

    /*Apply motion to src_cloud*/

    src_cloud = ((R_temp*src_cloud.transpose()).colwise() + t_temp).transpose();

  }
  std::cout << "Reaching Maximum Number of Iterations" << std::endl;
  result_cloud = src_cloud; result_R = R; result_t = t;
  result_T.block(0, 0, 3, 3) = R;
  result_T.block(0, 3, 3, 1) = t;
  result_T.block(3, 0, 1, 3).setZero();
  result_T(3, 3) = 1;
  return;
}

void scanMatching::pairing(my_kd_tree_t &my_tree, ICP::MatX &correspondances, bool sort, int Npo)
{
  nanoflann::KNNResultSet<double> result(1);

  size_t closest_index;
  double closest_sqdist;
  
  ICP::Vec3 query;

  // trying to align cloud 2 to cloud 1
  #pragma omp parallel for num_threads(8)
  for (size_t i = 0; i < src_cloud.rows(); i++)
  {
    query = src_cloud.row(i);

    result.init(&closest_index, &closest_sqdist);

    my_tree.index->findNeighbors(result, &query.x(), nanoflann::SearchParams());

    correspondances(i, 0) = closest_sqdist;
    correspondances(i, 1) = i;
    correspondances(i, 2) = closest_index;
  }

  if (sort){eigen_sort_rows_by_head(correspondances, Npo);}

}

void scanMatching::motion(ICP::MatX &dst, ICP::MatX &src,
                          ICP::Mat3 &R_temp, ICP::Vec3 &t_temp, ICP::Mat4 &T )
{
    /* Computing Means and Covariance*/
    Pmean = src.colwise().mean();

    Qmean = dst.colwise().mean();

    K = (src.transpose().colwise() - Pmean)*(dst.rowwise() - Qmean.transpose())/N;

    /*SVD Decomposition of Matrix K*/
    svd.compute(K);

    /*Computing R*/
    R_temp = svd.matrixV()*svd.matrixU().transpose();

    /*Computing T*/
    t_temp = Qmean - R_temp*Pmean;

    /* Formulating Transformation Matrix  934375 */
    T = ICP::Mat4::Zero();

    T.block(0, 0, 3, 3) = R_temp;
    T.block(0, 3, 3, 1) = t_temp;
    T(3, 3) = 1;
}

void scanMatching::rotationalError(SO3::Mat3 R, SO3::Mat3 Q)
{
  SO3::Vec3 v = SO3::Log(R.transpose()*Q);
  double rotational_angular_difference_RQ = v.norm(); // L2 norm .. in radians
  double rotational_angular_difference_RQ_deg = rotational_angular_difference_RQ / 3.1415926 * 180;
  std::cout << "Rotational Error (rad): " << rotational_angular_difference_RQ << std::endl;
  std::cout << "Rotational Error (deg):" << rotational_angular_difference_RQ_deg << std::endl;
}

void scanMatching::translationError(SO3::Vec3 t1, SO3::Vec3 t2)
{
  std::cout << "Translation Error:" << (t1 - t2).norm() << std::endl;
}

// http://nghiaho.com/?page_id=671

// http://www.inf.u-szeged.hu/ssip/2002/download/Chetverikov.pdf