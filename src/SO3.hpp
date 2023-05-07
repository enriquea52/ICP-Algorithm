#pragma once
#include "generic.h"

// number of decimals to allow for rounding a double
#define decimals 10000.0 

// Constant variable for the value of Pi
const double pi =  3.141592;

namespace SO3
{
    using Mat3 = Eigen::Matrix3d;
    using Vec3 = Eigen::Vector3d;
    
    // Action
    inline Vec3 Action(const Mat3& A, const Vec3& b){
        return A*b;
    }

    // Function used for rounding for a given number of decimals
    inline double roundDecimals(const double& number)
    {
        return std::round(number*decimals)/decimals;
    }

    // return a non-zero column from a given matrix
    inline Vec3 nonZeroCol(const Mat3& A)
    {
        double element;
        Vec3 v;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                element = roundDecimals(A(i, j));
                if (element != 0)
                {   
                    v << A(0, j), A(1, j), A(2, j);
                    return v;
                }
            }
        }
        return v;
    }

    // Exp map: angle -> Mat3
    inline Mat3 Exp(const Vec3& w) {
        // Implementation based on
        // http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf

        // Rodrigues formula expansion considering the rotation theta over an axis n =(nx, ny, nx)T

        double theta = w.norm();

        // Return identity matrix if the rotation is along a given axis is 0
        if (roundDecimals(theta) == 0.0){return Mat3::Identity();}

        Vec3 n = w/theta;

        Mat3 R;

        double c_theta = std::cos(theta), 
               s_theta = std::sin(theta),
               sub = (1 - c_theta);
         
        double m00 = c_theta + std::pow(n(0), 2)*sub;
        double m11 = c_theta + std::pow(n(1), 2)*sub;
        double m22 = c_theta + std::pow(n(2), 2)*sub;
        
        double m01 = (n(0)*n(1)*sub); double a01 = (n(2)*s_theta);
        double m02 = (n(0)*n(2)*sub); double a02 = (n(1)*s_theta);
        double m12 = (n(1)*n(2)*sub); double a12 = (n(0)*s_theta);

        R << m00, m01 - a01, m02 + a02,
             m01 + a01, m11, m12 - a12,
             m02 - a02, m12 + a12, m22;
        
        return R;
    }

    // Log map: SO(3) -> scalar
    inline Vec3 Log(const Mat3& R) {
        // Implementation  based on
        // https://courses.cs.duke.edu/fall13/compsci527/notes/rodrigues.pdf

        Mat3 A = (R - R.transpose())/2;
        Vec3 rho;
        Vec3 u, r;
        double theta;
        rho << A(2, 1), A(0, 2), A(1, 0);

        double s = rho.norm();
        double c = (R.trace() - 1)/2;

        s =  roundDecimals(s);
        c =  roundDecimals(c);

        // Whenever the Rotation Matrix = I, return r = 0 given that theta = 0
        if ((s == 0) && (c == 1)){return Vec3::Zero();}

        // In case multiple solutions may arise it is important to enforce uniquesness as follows
        if ((s == 0.0) && (c == -1.0))
        {
            Mat3 V = R + Mat3::Identity();
            Vec3 v = nonZeroCol(V);

            u = v/v.norm();
            r = u*pi;

            if (((roundDecimals(r.norm()) == roundDecimals(pi)) && 
                 (roundDecimals(r(0)) == 0 && roundDecimals(r(1)) == 0 && roundDecimals(r(2)) < 0)) || 
                 (roundDecimals(r(0)) && roundDecimals(r(1)) < 0) || 
                 (roundDecimals(r(0)) < 0)){return -1*r;}
            else {return r;}
        }

        // Finally if sin(theta) != 0, theta and u can be returned embedded as r = u*theta
        // where theta is the rotation angle along an axis u
        u = rho/s;
        theta = std::atan2(s, c);
        r = u*theta;

        return r;
    }

}