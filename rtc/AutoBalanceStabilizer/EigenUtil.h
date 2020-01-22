// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  EigenUtil.h
 * @brief
 * @date  $Date$
 */

#ifndef EIGENUTIL_H
#define EIGENUTIL_H

#include <Eigen/Geometry>

namespace hrp {

// To avoid zero division when normalizing. This problem has been resolved in the latest version of Eigen.
inline Eigen::VectorXd safeNormalize(const Eigen::VectorXd& vec)
{
    constexpr double EPS = 1e-6;
    const double norm = vec.norm();
    return (norm > EPS) ? vec.normalized() : vec;
}

inline Eigen::Matrix3d rotationMatrixFromOmega(const Eigen::Vector3d& omega)
{
    constexpr double EPS = 1e-6;
    const double norm = omega.norm();
    if (norm > EPS) return Eigen::AngleAxisd(norm, omega.normalized()).toRotationMatrix();
    else return Eigen::Matrix3d::Identity();
}

template<typename VEC>
inline VEC selectSmallVec(const VEC& vec1,
                          const VEC& vec2)
{
    return vec1.norm() < vec2.norm() ? vec1 : vec2;
}

Eigen::Matrix3d slerpMat(const Eigen::Ref<Eigen::Matrix3d>& start,
                         const Eigen::Ref<Eigen::Matrix3d>& goal,
                         const double ratio);

}

#endif // EIGENUTIL_H
