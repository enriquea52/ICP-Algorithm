#pragma once
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <Eigen/Dense>
#include <thread>
#include <random>


namespace ICP
{
    using MatX = Eigen::MatrixXd;
    using Mat4 = Eigen::Matrix4d;
    using Mat3 = Eigen::Matrix3d;
    using Vec3 = Eigen::Vector3d;
    using VecX = Eigen::VectorXd;
    using Afine3 = Eigen::Affine3d;
    using Axis = Eigen::AngleAxisd;
}
