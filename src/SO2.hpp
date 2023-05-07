#pragma once
#include <Eigen/Dense>

namespace SO2
{

    using Mat2 = Eigen::Matrix2d;
    using Vec2 = Eigen::Vector2d;

    // Action

    inline Vec2 Action(const Mat2& A, const Vec2& b){
        return A*b;
    }

    // Exp map: scalar -> Mat2
    inline Mat2 Exp(const double& phi) {
        constexpr double pi = 3.12159265;
        double angle = phi; // * pi / 180.0;
        double c = std::cos(angle), // rad in the parameter
               s = std::sin(angle); 
        return (Mat2() << c, -s, s, c).finished();
    }

    // Log map: SO(2) -> scalar
    inline double Log(const Mat2& R) {
        return std::atan2(R(1,0),R(0,0));
    }

    // log : SO(2) 

    // Log map: SO(2) -> scalar
    inline Mat2 Lie_log(const Mat2& R) {
        double phi = std::atan2(R(1,0),R(0,0));
        Mat2 r;
        r << 
            0.0, -phi,
            phi, 0.0;
        return r;
    }

    // boxplus: SO(2) x so(2) -> SO(2)
    // boxminus: SO(2) x SO(2) -> so(2) // note: "scalar" istead of so(2)


    inline Mat2 boxplus(const Mat2& X, const double& u)
    {
        return Exp(u) * X;
    }

    inline double boxminus(const Mat2& X, const Mat2& Y)
    {
        Log(X * Y.transpose());
    }

    // TODO: interpolate: SO(2) x SO(w) x scalar -> SO(2)

    inline Mat2 interpolate(const Mat2& X, const Mat2& Y, const double& t)
    {
        double u = boxminus(X, Y); // rotational difference 
        return boxplus(Y, t*u);
    }



}