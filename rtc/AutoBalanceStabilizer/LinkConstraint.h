// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LinkConstraint.h
 * @brief
 * @date  $Date$
 */

#ifndef LINKCONSTRAINT_H
#define LINKCONSTRAINT_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <Eigen/Geometry>
#include <hrpUtil/EigenTypes.h>
#include "../ImpedanceController/RatsMatrix.h"
#include "PlaneGeometry.h"

namespace hrp {

class LinkConstraint
{
  public:
    enum ConstraintType : size_t {FIX, FLOAT, FREE};

  private:
    int link_id; // TODO: Link id にするか Joint idにするか
    std::vector<hrp::Vector3> link_contact_points; // Link local
    Eigen::Isometry3d link_local_coord = Eigen::Isometry3d::Identity(); // Link local
    Eigen::Isometry3d target_coord = Eigen::Isometry3d::Identity(); // Global

    // TODO: sensorやref forceなど入れる？
    hrp::Vector3 cop_offset = hrp::Vector3::Zero(); // Link local
    double cop_weight = 1.0;
    ConstraintType constraint_type = FIX;

    // // IK variables
    // hrp::dvector6 constraint_weight = hrp::dvector6::Ones(); // Pos + Rot (Axis-angle)
    // double pos_precision = 1e-4;
    // double rot_precision = deg2rad(0.1);

  public:
    LinkConstraint(const int _link_id, const ConstraintType _constraint_type = FIX) // TODO: FLOATではweightを0に?
        : link_id(_link_id), constraint_type(_constraint_type) {}

    int getLinkId() const { return link_id; }
    const std::vector<hrp::Vector3>& getLinkContactPoints() const { return link_contact_points; }

    void addLinkContactPoint(const hrp::Vector3& pos)
    {
        link_contact_points.push_back(pos);
    }

    void clearLinkContactPoints()
    {
        link_contact_points.clear();
    }

    void calcLinkLocalPos()
    {
        link_local_coord.translation() = std::accumulate(link_contact_points.begin(), link_contact_points.end(),
                                                         hrp::Vector3::Zero().eval())
            / link_contact_points.size() + cop_offset;
    }
    Eigen::Isometry3d::ConstTranslationPart localPos() const { return link_local_coord.translation(); }

    Eigen::Isometry3d::LinearPart localRot() { return link_local_coord.linear(); }
    Eigen::Isometry3d::ConstLinearPart localRot() const { return link_local_coord.linear(); }

    Eigen::Isometry3d& targetCoord() { return target_coord; }
    const Eigen::Isometry3d& targetCoord() const { return target_coord; }

    Eigen::Isometry3d::TranslationPart targetPos() { return target_coord.translation(); }
    Eigen::Isometry3d::ConstTranslationPart targetPos() const { return target_coord.translation(); }

    Eigen::Isometry3d::LinearPart targetRot() { return target_coord.linear(); }
    Eigen::Isometry3d::ConstLinearPart targetRot() const { return target_coord.linear(); }

    void setCOPWeight(double _weight) { cop_weight = _weight; }
    double getCOPWeight() const { return cop_weight; }

    void setConstraintType(ConstraintType type) { constraint_type = type; }
    ConstraintType getConstraintType() const { return constraint_type; }

    std::vector<Eigen::Vector2d> calcContactConvexHull() const
    {
        std::vector<Eigen::Vector2d> vertices;
        vertices.reserve(link_contact_points.size());
        Eigen::Isometry3d::ConstTranslationPart target_pos = targetPos();
        Eigen::Isometry3d::ConstLinearPart target_rot = targetRot();
        for (const hrp::Vector3& point : link_contact_points) {
            vertices.push_back((target_pos - target_rot * (link_local_coord.translation() + point)).head<2>()); // TODO: rot確認
        }
        return calcConvexHull(vertices); // TODO: 1点のときと2点のときの確認
    }

    hrp::Vector3 calcActualTargetPosFromLinkState(const hrp::Vector3& link_pos, const hrp::Matrix33& link_rot) const
    {
        return link_pos + link_rot * localPos();
    }

    hrp::Matrix33 calcActualTargetRotFromLinkState(const hrp::Matrix33& link_rot) const
    {
        return link_rot * localRot();
    }
};

struct ConstraintsWithCount
{
    std::vector<LinkConstraint> constraints;
    size_t start_count = 0;

    hrp::Vector3 calcCOPFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const
    {
        hrp::Vector3 cop_pos = hrp::Vector3::Zero();
        double sum_weight = 0;

        for (const LinkConstraint& constraint : constraints) {
            if (constraint.getConstraintType() >= type_thre) continue;
            const double weight = constraint.getCOPWeight();
            // TODO:
            //       面接触してる時みたいに、target_coordと実際がずれているときは？
            cop_pos += constraint.targetPos() * weight;
            sum_weight += weight;
        }
        if (sum_weight > 0) cop_pos /= sum_weight;

        return cop_pos;
    }

    hrp::Matrix33 calcCOPRotationFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const
    {
        Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
        double sum_weight = 0;

        for (const LinkConstraint& constraint : constraints) {
            const double weight = constraint.getCOPWeight();
            if (constraint.getConstraintType() >= type_thre || weight == 0 /* to avoid zero division */) continue;
            sum_weight += weight;
            const Eigen::Quaternion<double> contact_quat(constraint.targetRot());
            cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
        }

        return cop_quat.toRotationMatrix();
    }

    Eigen::Isometry3d calcCOPCoord(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const
    {
        Eigen::Isometry3d coord;
        coord.translation() = calcCOPFromConstraints(type_thre);
        coord.linear() = calcCOPRotationFromConstraints(type_thre);
        return coord;
    }

    int getConstraintIndexFromLinkId(const int id) const
    {
        for (size_t idx = 0; idx < constraints.size(); ++idx) {
            if (constraints[idx].getLinkId() == id) return idx;
        }

        std::cerr << "[LinkConstraint] Can't find constraint for " << id << " th link" << std::endl;
        return -1;
    }

    std::vector<size_t> getConstraintIndicesFromType(const LinkConstraint::ConstraintType type) const
    {
        std::vector<size_t> indices;
        indices.reserve(constraints.size());
        for (size_t i = 0; i < constraints.size(); ++i) {
            if (constraints[i].getConstraintType() == type) indices.push_back(i);
        }
        return indices;
    }

    std::vector<Eigen::Vector2d> calcContactConvexHullForAllConstraints() const
    {
        std::vector<Eigen::Vector2d> vertices;
        for (const LinkConstraint& constraint : constraints) {
            if (constraint.getConstraintType() >= LinkConstraint::FLOAT) continue;

            Eigen::Isometry3d::ConstTranslationPart target_pos = constraint.targetPos();
            Eigen::Isometry3d::ConstLinearPart target_rot = constraint.targetRot();
            const hrp::Vector3& link_local_pos = constraint.localPos();
            for (const hrp::Vector3& point : constraint.getLinkContactPoints()) {
                vertices.push_back((target_pos - target_rot * (link_local_pos + point)).head<2>()); // TODO: rot
            }
        }
        return calcConvexHull(vertices);
    }
};


// Utility functions

inline size_t getConstraintIndexFromCount(const std::vector<ConstraintsWithCount>& constraints_list,
                                          const size_t count, const size_t start_idx = 0)
{
    size_t constraint_index = start_idx;
    for (size_t i = start_idx; i < constraints_list.size() && constraints_list[i].start_count <= count; ++i) {
        constraint_index = i;
    }
    return constraint_index;
}

}

#endif // LINKCONSTRAINT_H
