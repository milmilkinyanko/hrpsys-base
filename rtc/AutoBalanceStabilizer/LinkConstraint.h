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

namespace hrp
{

class LinkConstraint
{
  public:
    enum ConstraintType {FIX, FLOAT, FREE};

  private:
    int link_id;
    std::vector<hrp::Vector3> link_contact_points; // Link local
    hrp::Vector3 link_representative_point = hrp::Vector3::Zero(); // Link local
    Eigen::AffineCompact3d transformation_matrix; // transformation from world
    Eigen::AffineCompact3d target_coord = Eigen::AffineCompact3d::Identity(); // TODO: local or global

    hrp::Vector3 cop_offset = hrp::Vector3::Zero(); // local or global
    double weight = 1.0;
    ConstraintType constraint_type = FIX;

    size_t next_turning_count; // TODO: 着地のカウントなど，補間に使うカウントを保存
    // LimbTrajectoryGenerator ltg; // TODO: 別にvectorかmapで持ったほうが良い? => TrajectoryTypeの変更周りが微妙になりそう

    // TODO: pos, vel, accなどlimbの状態を表す変数を持つクラスを上位に作って，それがcontactを持つクラスを持つ？微妙
    // Robot local or global
    hrp::Vector3 pos;
    hrp::Vector3 vel;
    hrp::Vector3 acc;

  public:
    LinkConstraint(const int _link_id, const ConstraintType _constraint_type = FIX) // TODO: FLOATではweightを0に
        : link_id(_link_id), constraint_type(_constraint_type) {}

    void copyLimbStates(const LinkConstraint& other)
    {
        // ltg = other.ltg;
        pos = other.pos;
        vel = other.vel;
        acc = other.acc;
    }

    const int getLinkId() const { return link_id; }
    const std::vector<hrp::Vector3>& getLinkContactPoints() const { return link_contact_points; }

    void addLinkContactPoint(const hrp::Vector3& pos)
    {
        link_contact_points.push_back(pos);
    }

    void clearLinkContactPoints()
    {
        link_contact_points.clear();
    }

    void calcLinkRepresentativePoint()
    {
        link_representative_point = std::accumulate(link_contact_points.begin(), link_contact_points.end(), hrp::Vector3::Zero().eval())
            / link_contact_points.size() + cop_offset;
    }
    const hrp::Vector3& getLinkRepresentativePoint() const { return link_representative_point; }

    // Eigen::AffineCompact3d& envRepresentativeCoord() { return env_representative_coord; }
    // const Eigen::AffineCompact3d& envRepresentativeCoord() const { return env_representative_coord; }
    Eigen::AffineCompact3d& targetCoord() { return target_coord; } // TODO: use affine() ?
    const Eigen::AffineCompact3d& targetCoord() const { return target_coord; }

    Eigen::AffineCompact3d::TranslationPart targetPos() { return target_coord.translation(); }
    Eigen::AffineCompact3d::ConstTranslationPart targetPos() const { return target_coord.translation(); }

    Eigen::AffineCompact3d::LinearPart targetRot() { return target_coord.linear(); }
    Eigen::AffineCompact3d::ConstLinearPart targetRot() const { return target_coord.linear(); }

    void setWeight(double _weight) { weight = _weight; }
    double getWeight() const { return weight; }

    void setConstraintType(ConstraintType type) { constraint_type = type; }
    ConstraintType getConstraintType() const { return constraint_type; }

    void setTransformationMatrix(const Eigen::AffineCompact3d& trans)
    {
        transformation_matrix = trans;
    }

    void setTransformationMatrix(const hrp::Vector3& pos, const hrp::Matrix33& rot)
    {
        transformation_matrix.translation() = pos;
        transformation_matrix.linear() = rot;
    }

    const Eigen::AffineCompact3d& getTransformationMatrix() const
    {
        return transformation_matrix;
    }

    // TODO: 座標変換 ?
    const hrp::Vector3& getPos() const { return pos; }
    const hrp::Vector3& getVel() const { return vel; }
    const hrp::Vector3& getAcc() const { return acc; }

    void calcEnvironmentRepresentativeCoordFromContacts(const std::vector<Eigen::AffineCompact3d>& coords);

    Eigen::AffineCompact3d calcRepresentativeCoord(const std::vector<hrp::Vector3>& points, const hrp::Matrix33& rot);

    void rotateLinkRot(const hrp::Matrix33& rot);

    const Eigen::AffineCompact3d calcTransformedCoord(const Eigen::AffineCompact3d& coord); // TODO: ここか？
};

class ConstraintsWithCount
{
  public:
    std::vector<LinkConstraint> constraints;
    size_t start_count = 0;

    hrp::Vector3 calcCOPFromConstraints() const
    {
        hrp::Vector3 cop_pos = hrp::Vector3::Zero();
        double sum_weight = 0;

        for (const LinkConstraint& constraint : constraints) {
            if (constraint.getConstraintType() >= LinkConstraint::FLOAT) continue;
            const double weight = constraint.getWeight();
            // TODO: 座標変換
            cop_pos += constraint.getLinkRepresentativePoint() * weight; // TODO: representative pointを使うかどうか
            sum_weight += weight;
        }
        cop_pos /= sum_weight;

        return cop_pos;
    }

    hrp::Matrix33 calcCOPRotationFromConstraints() const
    {
        Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
        double sum_weight = 0;

        for (const LinkConstraint& constraint : constraints) {
            const double weight = constraint.getWeight();
            if (constraint.getConstraintType() >= LinkConstraint::FLOAT || weight == 0 /* to avoid zero division */) continue;
            sum_weight += weight;
            const Eigen::Quaternion<double> contact_quat(constraint.targetRot());
            cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
        }

        return cop_quat.toRotationMatrix();
    }

    int getConstraintIndexFromLinkId(const int id) const
    {
        for (int idx = 0; idx < constraints.size(); ++idx) {
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
};


// Utility functions

inline size_t getCurrentConstraintIndex(const std::vector<ConstraintsWithCount>& constraints_list,
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
