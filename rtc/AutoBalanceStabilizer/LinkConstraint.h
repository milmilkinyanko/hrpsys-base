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
#include <memory>
#include <Eigen/Geometry>
#include <hrpUtil/EigenTypes.h>
#include "../ImpedanceController/RatsMatrix.h"
#include "LimbTrajectoryGenerator.h"

namespace hrp {

// class LimbTrajectoryGenerator;

class LinkConstraint
{
  public:
    // enum ConstraintType : size_t {FIX, TOE_FORWARD, TOE_BACKWARD, HEEL_FORWARD, HEEL_BACKWARD, FLOAT, FREE};
    enum ConstraintType : size_t {FIX, FLOAT, FREE};

  private:
    int link_id;
    std::vector<hrp::Vector3> link_contact_points; // Link local
    Eigen::Isometry3d link_local_coord = Eigen::Isometry3d::Identity(); // Link local
    Eigen::Isometry3d target_coord = Eigen::Isometry3d::Identity(); // Global

    // TODO: sensorやref forceなど入れる？
    hrp::Vector3 cop_offset = hrp::Vector3::Zero(); // Link local
    double cop_weight = 1.0; // TODO: FLOATではweightを0に?
    ConstraintType constraint_type = FIX;

    // // IK variables
    // hrp::dvector6 constraint_weight = hrp::dvector6::Ones(); // Pos + Rot (Axis-angle)
    // double pos_precision = 1e-4;
    // double rot_precision = deg2rad(0.1);

    // TODO: この辺はGaitGeneratorが持つべきか
    std::vector<hrp::Vector3> default_contact_points; // Link local
    std::vector<hrp::Vector3> toe_contact_points; // Link local
    std::vector<hrp::Vector3> heel_contact_points; // Link local

    // TODO: つま先離陸，かかと着陸の時の補間

    LimbTrajectoryGenerator limb_traj;
    // std::unique_ptr<LimbTrajectoryGenerator> limb_traj_impl;

  public:
    LinkConstraint(const int _link_id, const ConstraintType _constraint_type = FIX);
    // LinkConstraint(const LinkConstraint& lc);
    // LinkConstraint& operator=(const LinkConstraint& lc);
    // ~LinkConstraint();
    // LinkConstraint(LinkConstraint&& lc);
    // LinkConstraint& operator=(LinkConstraint&& lc);

    int getLinkId() const { return link_id; }
    const std::vector<hrp::Vector3>& getLinkContactPoints() const { return link_contact_points; }

    void addLinkContactPoint(const hrp::Vector3& pos) { link_contact_points.push_back(pos); }
    void clearLinkContactPoints() { link_contact_points.clear(); }

    void calcLinkLocalPos()
    {
        link_local_coord.translation() =
            std::accumulate(link_contact_points.begin(),
                            link_contact_points.end(),
                            hrp::Vector3::Zero().eval()) / link_contact_points.size() + cop_offset;
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

    std::vector<Eigen::Vector2d> calcContactConvexHull() const;

    hrp::Vector3 calcActualTargetPosFromLinkState(const hrp::Vector3& link_pos, const hrp::Matrix33& link_rot) const
    {
        return link_pos + link_rot * localPos();
    }

    hrp::Matrix33 calcActualTargetRotFromLinkState(const hrp::Matrix33& link_rot) const
    {
        return link_rot * localRot();
    }

    // LimbTrajectoryGenerator
    void copyLimbTrajectoryGenerator(const LinkConstraint& lc) { limb_traj = lc.limb_traj; }
    void copyLimbState(const LinkConstraint& lc)
    {
        if (limb_traj.isViaPointsEmpty()) limb_traj = lc.limb_traj;
        else limb_traj.copyState(lc.limb_traj);
    }
    void clearLimbViaPoints() { limb_traj.clearViaPoints(); }
    void calcLimbViaPoints(const LimbTrajectoryGenerator::TrajectoryType traj_type,
                           const Eigen::Isometry3d& goal,
                           const size_t start_count, const size_t goal_count,
                           const double step_height)
    {
        limb_traj.calcViaPoints(traj_type, target_coord, goal, start_count, goal_count, step_height);
    }
    void calcLimbTrajectory(const size_t cur_count, const double dt)
    {
        if (!limb_traj.isViaPointsEmpty()) {
            limb_traj.calcTrajectory(cur_count, dt);
            targetPos() = limb_traj.getPos();
            targetRot() = limb_traj.getRot();
        }
    }
};

struct ConstraintsWithCount
{
    std::vector<LinkConstraint> constraints;
    size_t start_count = 0;

    hrp::Vector3 calcCOPFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const;
    hrp::Matrix33 calcCOPRotationFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const;
    Eigen::Isometry3d calcCOPCoord(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const
    {
        Eigen::Isometry3d coord;
        coord.translation() = calcCOPFromConstraints(type_thre);
        coord.linear() = calcCOPRotationFromConstraints(type_thre);
        return coord;
    }

    int getConstraintIndexFromLinkId(const int id) const;
    std::vector<size_t> getConstraintIndicesFromType(const LinkConstraint::ConstraintType type) const;

    std::vector<Eigen::Vector2d> calcContactConvexHullForAllConstraints() const;

    // LimbTrajectoryGenerator
    void copyLimbTrajectoryGenerator(const ConstraintsWithCount& cwc);
    void copyLimbState(const ConstraintsWithCount& cwc);
    void clearLimbViaPoints()
    {
        for (auto& constraint : constraints) constraint.clearLimbViaPoints();
    }
    void calcLimbTrajectory(const size_t cur_count, const double dt)
    {
        for (auto& constraint : constraints) constraint.calcLimbTrajectory(cur_count, dt);
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
