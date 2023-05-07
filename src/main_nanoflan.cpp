#include "../nanoflann.hpp"

#include <cstdlib>
#include <ctime>
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace nanoflann;

const int SAMPLES_DIM = 15;

template <typename Der>
void generateRandomPointCloud(Eigen::MatrixBase<Der> &mat, const size_t N,
                              const size_t dim,
                              const typename Der::Scalar max_range = 10) {
  std::cout << "Generating " << N << " random points...";
  mat.resize(N, dim);
  for (size_t i = 0; i < N; i++)
    for (size_t d = 0; d < dim; d++)
      mat(i, d) = max_range * (rand() % 1000) / typename Der::Scalar(1000);
  std::cout << "done\n";
}

template <typename num_t>
void kdtree_demo(const size_t nSamples, const size_t dim) {

  Eigen::Matrix<num_t, Dynamic, Dynamic> mat(nSamples, dim);

  const num_t max_range = 20;

  // Generate points:
  generateRandomPointCloud(mat, nSamples, dim, max_range);

  //	cout << mat << endl;

  // Query point:
  std::vector<num_t> query_pt(dim);
  for (size_t d = 0; d < dim; d++)
    query_pt[d] = max_range * (rand() % 1000) / num_t(1000);

  // ------------------------------------------------------------
  // construct a kd-tree index:
  //    Some of the different possibilities (uncomment just one)
  // ------------------------------------------------------------
  // Dimensionality set at run-time (default: L2)
  //typedef KDTreeEigenMatrixAdaptor<Eigen::Matrix<num_t, Dynamic, Dynamic>>
  //    my_kd_tree_t;

  // Dimensionality set at compile-time
  //	typedef KDTreeEigenMatrixAdaptor< Eigen::Matrix<num_t,Dynamic,Dynamic> >
  // my_kd_tree_t;

  // Dimensionality set at compile-time: Explicit selection of the distance
  // metric: L2
typedef KDTreeEigenMatrixAdaptor<
     Eigen::Matrix<num_t,Dynamic,Dynamic>, Eigen::Dynamic, nanoflann::metric_L2> 
    my_kd_tree_t;
  // Dimensionality set at compile-time: Explicit selection of the distance
  // metric: L2_simple
  //	typedef KDTreeEigenMatrixAdaptor<
  // Eigen::Matrix<num_t,Dynamic,Dynamic>,nanoflann::metric_L2_Simple>
  // my_kd_tree_t;

  // Dimensionality set at compile-time: Explicit selection of the distance
  // metric: L1
  //	typedef KDTreeEigenMatrixAdaptor<
  // Eigen::Matrix<num_t,Dynamic,Dynamic>,nanoflann::metric_L1>  my_kd_tree_t;

  my_kd_tree_t mat_index(dim, std::cref(mat), 10 /* max leaf */);
  //  this will take some (brief) time 
  mat_index.index->buildIndex();

  // do a knn search
  const size_t num_results = 3; // 3 nn
  vector<size_t> ret_indexes(num_results);
  vector<num_t> out_dists_sqr(num_results);

  nanoflann::KNNResultSet<num_t> resultSet(num_results);
  resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

  mat_index.index->findNeighbors(resultSet, &query_pt[0],
                                 nanoflann::SearchParams(10));

  std::cout << "knnSearch(nn=" << num_results << "): \n";
  for (size_t i = 0; i < num_results; i++)
    std::cout << "ret_index[" << i << "]=" << ret_indexes[i]
              << " out_dist_sqr=" << out_dists_sqr[i] << endl;
}

void circle_demo()
{
  using PC_type = Eigen::Matrix<float, Eigen::Dynamic, 2>;

  PC_type pc;
  pc.resize(100, Eigen::NoChange);

  float radius = 1.0;

  for (size_t i = 0; i < pc.rows(); i++)
  {
    float phi = static_cast<float>(i) / pc.rows() * 2 * 3.14159265;
    pc.row(i) = radius * Eigen::Vector2f(std::cos(phi), std::sin(phi));
  }

  using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
  PC_type, 2, nanoflann::metric_L2>;

  my_kd_tree_t my_tree(2 /*PC_type::ColsAtCompileTime*/, std::cref(pc));
  my_tree.index->buildIndex();

  nanoflann::KNNResultSet<float> result(1);

  size_t closest_index;
  float closest_sqdist;
  

  for (size_t i = 0; i < 20; i++)
  {
    float phi = static_cast<float>(i) / 20 * 2 * 3.14159265;
  
    Eigen::Vector2f query;
    query = radius * Eigen::Vector2f(std::cos(phi), std::sin(phi));
    result.init(&closest_index, &closest_sqdist);

    my_tree.index->findNeighbors(result, &query.x(), nanoflann::SearchParams());
    
    std::cout << closest_index << std::endl;

  }



}

int main(int argc, char **argv) {
  // Randomize Seed
  srand(static_cast<unsigned int>(time(nullptr)));
  // kdtree_demo<float>(1000 /* samples */, SAMPLES_DIM /* dim */);
   circle_demo();
  return 0;
}