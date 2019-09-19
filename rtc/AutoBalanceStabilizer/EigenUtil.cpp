// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  EigenUtil.cpp
 * @brief
 * @date  $Date$
 */

#include "EigenUtil.h"

namespace hrp {

Eigen::Matrix3d slerpMat(const Eigen::Ref<Eigen::Matrix3d>& start,
                         const Eigen::Ref<Eigen::Matrix3d>& goal,
                         const double ratio)
{
    const Eigen::Quaternion<double> start_quat(start);
    const Eigen::Quaternion<double> goal_quat(goal);
    return start_quat.slerp(ratio, goal_quat).toRotationMatrix();
}

}
