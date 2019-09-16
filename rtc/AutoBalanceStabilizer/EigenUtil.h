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

inline Eigen::Isometry3d transform(const Eigen::Isometry3d& target,
                                   const Eigen::Isometry3d& target_origin,
                                   const Eigen::Isometry3d& new_origin)
{
    return new_origin.inverse() * target_origin * target;
}

}

#endif // EIGENUTIL_H
