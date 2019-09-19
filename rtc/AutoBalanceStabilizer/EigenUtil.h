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

Eigen::Matrix3d slerpMat(const Eigen::Ref<Eigen::Matrix3d>& start,
                         const Eigen::Ref<Eigen::Matrix3d>& goal,
                         const double ratio);

}

#endif // EIGENUTIL_H
