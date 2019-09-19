// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  Utility.h
 * @brief
 * @date  $Date$
 */

#ifndef ABST_UTILITY_H
#define ABST_UTILITY_H

#include <cmath>
#include <hrpModel/Link.h>
#include <hrpModel/Body.h>

namespace hrp {

constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }

inline bool eps_eq(const double a, const double b, const double eps = 0.001)
{
    return fabs(a - b) <= eps;
}

inline double calcInteriorPoint(const double start, const double goal, const double ratio)
{
    return (1 - ratio) * start + ratio * goal;
}

template<typename T>
inline T calcInteriorPoint(const T& start, const T& goal, const double ratio)
{
    return (1 - ratio) * start + ratio * goal;
}

inline double clamp(const double value, const double llimit_value, const double ulimit_value)
{
    return std::max(llimit_value, std::min(ulimit_value, value));
}

inline hrp::Vector3 clamp(const hrp::Vector3& value, const double llimit_value, const double ulimit_value)
{
    return value.array().max(llimit_value).min(ulimit_value);
}

inline hrp::Vector3 clamp(const hrp::Vector3& value, const hrp::Vector3& limit_value)
{
    return value.array().max(-limit_value.array()).min(limit_value.array());
}

inline hrp::Vector3 clamp(const hrp::Vector3& value, const hrp::Vector3& llimit_value, const hrp::Vector3& ulimit_value)
{
    return value.array().max(llimit_value.array()).min(ulimit_value.array());
}

inline void copyJointAnglesToRobotModel(const hrp::BodyPtr& m_robot,
                                        const hrp::dvector& joint_angles)
{
    const size_t num_joints = m_robot->numJoints();
    assert(num_joints <= joint_angles.size());
    for (size_t i = 0; i < num_joints; ++i) {
        m_robot->joint(i)->q = joint_angles[i];
    }
}

inline void copyJointAnglesFromRobotModel(hrp::dvector& joint_angles,
                                          const hrp::BodyPtr& m_robot)
{
    const size_t num_joints = m_robot->numJoints();
    joint_angles.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        joint_angles[i] = m_robot->joint(i)->q;
    }
}

}

#endif // ABST_UTILITY_H
