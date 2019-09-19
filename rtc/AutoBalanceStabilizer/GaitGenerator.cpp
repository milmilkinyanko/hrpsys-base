// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  GaitGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <boost/make_shared.hpp>
#include "GaitGenerator.h"
#include "Utility.h"
#include "EigenUtil.h"

namespace hrp {

GaitGenerator::GaitGenerator(const hrp::BodyPtr& _robot,
                             const double _dt,
                             std::vector<LinkConstraint>&& init_constraints,
                             const double preview_time) :
    root_id(_robot->rootLink()->index),
    preview_window(static_cast<size_t>(std::round(preview_time / _dt)))
{
    const size_t num_limbs = init_constraints.size();

    // Set initial position as target (need to be done calcForwardKinematics)
    // TODO: ここで良いか
    for (auto& constraint : init_constraints) {
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        constraint.targetPos() = constraint.calcActualTargetPosFromLinkState(link->p, link->R);
        std::cerr << "link   " << link->name << " pos: " << link->p.transpose() << std::endl;
        std::cerr << "target " << link->name << " pos: " << constraint.targetPos().transpose() << std::endl;
        std::cerr << "local  " << link->name << " pos: " << constraint.localPos().transpose() << std::endl;
        constraint.targetRot() = constraint.calcActualTargetRotFromLinkState(link->R);
    }

    limb_gens.resize(num_limbs);
    for (size_t i = 0; i < num_limbs; ++i) {
        limb_gens[i].setPos(static_cast<hrp::Vector3>(init_constraints[i].targetPos()));
    }

    constraints_list.resize(1);
    constraints_list[0].constraints = std::move(init_constraints); // TODO: これでmoveできるかテストする
    constraints_list[0].start_count = 0;

    root_coord.translation() = _robot->rootLink()->p;
    root_coord.linear() = _robot->rootLink()->R;

    prev_ref_cog = _robot->calcCM();
    zmp_gen = std::make_unique<RefZMPGenerator>(_dt, preview_window, constraints_list[0]);
    cog_gen = std::make_unique<COGTrajectoryGenerator>(prev_ref_cog);

    cog_gen->initPreviewController(_dt, zmp_gen->getCurrentRefZMP());
}

void GaitGenerator::forwardTimeStep(const size_t cur_count)
{
    zmp_gen->popAndPushRefZMP(constraints_list, cur_count);
    cur_const_idx = getConstraintIndexFromCount(constraints_list, cur_count);

    // TODO: 関数化?
    if (cur_count == constraints_list[cur_const_idx].start_count) {
        for (size_t i = 0; i < constraints_list[cur_const_idx].constraints.size(); ++i) {
            if (constraints_list[cur_const_idx].constraints[i].getConstraintType() == hrp::LinkConstraint::FLOAT) {
                limb_gens[i].calcViaPoints(default_traj_type, constraints_list,
                                           constraints_list[cur_const_idx].constraints[i].getLinkId(),
                                           cur_count, default_step_height);
            }
        }
    }
}

void GaitGenerator::calcCogAndLimbTrajectory(const size_t cur_count, const double dt)
{
    cog_gen->calcCogFromZMP(zmp_gen->getRefZMPList());

    for (auto& limb_gen : limb_gens) {
        limb_gen.calcTrajectory(cur_count, dt);
    }

    for (size_t i = 0; i < limb_gens.size(); ++i) {
        constraints_list[cur_const_idx].constraints[i].targetPos() = limb_gens[i].getPos();
        constraints_list[cur_const_idx].constraints[i].targetRot() = limb_gens[i].getRot(); // TODO
    }

    root_coord.translation() += cog_gen->getCog() - prev_ref_cog;
    prev_ref_cog = cog_gen->getCog();

    return;
}

// This function is almost the same as ConstraintsWithCount.calcCOPFromConstraints()
hrp::Vector3 GaitGenerator::calcReferenceCOPFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const
{
    hrp::Vector3 cop_pos = hrp::Vector3::Zero();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : cur_consts) {
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT) continue;
        const double weight = constraint.getCOPWeight();
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        cop_pos += constraint.calcActualTargetPosFromLinkState(link->p, link->R) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0) cop_pos /= sum_weight;

    return cop_pos;
}

// This function is almost the same as ConstraintsWithCount.calcCOPRotationFromConstraints()
hrp::Matrix33 GaitGenerator::calcReferenceCOPRotFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const
{
    Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : cur_consts) {
        const double weight = constraint.getCOPWeight();
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT || weight == 0 /* to avoid zero division */) continue;
        sum_weight += weight;
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        const Eigen::Quaternion<double> contact_quat(constraint.calcActualTargetRotFromLinkState(link->R));
        cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
    }

    return cop_quat.toRotationMatrix();
}

void GaitGenerator::adjustCOPCoordToTarget(const hrp::BodyPtr& _robot, const size_t count)
{
    const ConstraintsWithCount& cur_consts = getCurrentConstraints(count);
    const hrp::Vector3 cop_pos = calcReferenceCOPFromModel(_robot, cur_consts.constraints);
    const hrp::Matrix33 cop_rot = calcReferenceCOPRotFromModel(_robot, cur_consts.constraints);

    const hrp::Vector3 target_cop_pos = cur_consts.calcCOPFromConstraints();
    const hrp::Matrix33 target_cop_rot = cur_consts.calcCOPRotationFromConstraints();

    rootPos() = (target_cop_pos - cop_pos) + _robot->rootLink()->p;
    rootRot() = target_cop_rot * cop_rot.transpose() * _robot->rootLink()->R;
}

// TODO: foot stepの生成はm_robotを持っている上位に移すべきか
void GaitGenerator::addNextFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                                const size_t swing_start_count, const double step_time, const double dt,
                                                const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                                const int support_id, const int swing_id)
{
    const size_t cur_idx = getConstraintIndexFromCount(constraints_list, swing_start_count - 1);
    const int support_idx = constraints_list[cur_idx].getConstraintIndexFromLinkId(support_id);
    const int swing_idx   = constraints_list[cur_idx].getConstraintIndexFromLinkId(swing_id);

    if (support_idx == -1 || swing_idx == -1) {
        std::cerr << "[GaitGenerator] Cannot find constraint " << support_id << "or constraint " << swing_id << std::endl; // TODO: error message
        return;
    }

    constraints_list.reserve(constraints_list.size() + 2);

    {
        ConstraintsWithCount swing_phase_constraints(constraints_list[cur_idx]);
        swing_phase_constraints.start_count = swing_start_count;
        swing_phase_constraints.constraints[support_idx].setConstraintType(LinkConstraint::FIX);
        swing_phase_constraints.constraints[swing_idx].setConstraintType(LinkConstraint::FLOAT);
        constraints_list.push_back(std::move(swing_phase_constraints));
    }

    {
        ConstraintsWithCount landing_phase_constraints(constraints_list[cur_idx]);
        landing_phase_constraints.start_count = swing_start_count + static_cast<size_t>(step_time / dt);
        LinkConstraint& support = landing_phase_constraints.constraints[support_idx];
        LinkConstraint& landing = landing_phase_constraints.constraints[swing_idx];
        support.setConstraintType(LinkConstraint::FIX);
        landing.setConstraintType(LinkConstraint::FIX);

        // Support limb coordinate
        double stride_x = vel_x / step_time;
        double stride_y = vel_y / step_time + (support.targetRot().inverse() * (landing.targetPos() - support.targetPos()))[1];
        double yaw_angle = vel_yaw / step_time;

        // TODO: stride limitation

        Eigen::Isometry3d trans_sup_local;
        trans_sup_local.translation() = Eigen::Vector3d(stride_x, stride_y, 0);
        trans_sup_local.linear() = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        landing.targetCoord() = trans_sup_local * support.targetCoord();

        constraints_list.push_back(std::move(landing_phase_constraints));
    }
}

// // Rot は world base rotを使う ?
// void GaitGenerator::goPosBiped(const Eigen::Isometry3d& target, const size_t one_step_count,
//                                const double max_step_length, const double max_rotate_angle /*[rad]*/,
//                                const std::vector<int>& support_link_cycle,
//                                const std::vector<int>& swing_link_cycle)
// {
//     const ConstraintsWithCount& cur_constraints = getCurrentConstraints(count);

//     const hrp::Vector3 init_cop = cur_constraints.calcCOPFromConstraints();
//     const double distance = (init_cop - target.translation()).head<2>().norm();
//     const hrp::Matrix33 diff_rot = cur_constraints.calcCOPRotationFromConstraints().transpose() * target.linear();
//     const double diff_yaw = rpyFromRot(diff_rot)(2);

//     const size_t one_cycle_len = support_link_cycle.size();
//     const size_t translation_steps = static_cast<size_t>(std::ceil(distance / max_step_length));
//     size_t rotation_steps = static_cast<size_t>(std::ceil(diff_yaw / max_rotate_angle));
//     rotation_steps += rotation_steps % one_cycle_len;
//     // size_t num_steps = std::max(translation_steps, rotation_steps);
//     num_steps = translation_steps; // TODO: tmp straight
//     num_steps += num_steps % one_cycle_len;
//     const size_t num_cycle = num_steps / one_cycle_len;

//     constraints_list.resize(num_steps + 2);
//     constraints_list[0] = cur_constraints;

//     const hrp::Vector3 one_translation_step = (init_cop - target.translation()) / translation_steps;

//     // std::unordered_map<int, Eigen::Isometry3d> one_rotation_steps; // 自転, weightが左右異なる場合，左右で違う行列となる
//     // for (const LinkConstraint& constraint : cur_constraints.constraints) {
//     //     // TODO: 1歩の回転の計算
//     // }

//     for (size_t i = 0; i < num_cycle; ++i) {
//         for (size_t j = 0; j < one_cycle_len; ++j) {
//             constraints_list[i + j + 1] = constraints_list[i + j];
//             ConstraintsWithCount& next_constraints = constraints_list[i + j + 1];

//             const int support_idx = cur_constraints.getConstraintIndexFromLinkId(support_id);
//             const int swing_idx   = cur_constraints.getConstraintIndexFromLinkId(swing_id);

//             if (support_idx == -1 || swing_idx == -1) {
//                 std::cerr << "Error" << std::endl; // TODO: error message
//                 return;
//             }

//             LinkConstraint& support = cur_constraints.constraints[support_idx];
//             LinkConstraint& swing   = cur_constraints.constraints[swing_idx];

//             // LinkConstraint& support = next_constraints.getConstraintsFromLinkIds(support_link_cycle[j])[0];
//             // LinkConstraint& swing = next_constraints.getConstraintsFromLinkIds(swing_link_cycle[j])[0];

//             swing.targetCoord().translation() = support.targetCoord().translation() + one_translation_step;
//             // swing.targetCoord().linear() = support.targetCoord().linear(); // 1cycleごとに両足が揃うのか，片方の足を追い越して進んでいくのか
//         }
//     }
// }

// void GaitGenerator::goPos(const rats::coordinates& target, const size_t one_step_count,
//                           const double max_step_length, const double max_rotate_angle,
//                           const std::vector<std::vector<int>>& support_link_cycle,
//                           const std::vector<std::vector<int>>& swing_link_cycle)
// {
//     getCurrentConstraints().calcCOPFromConstraints();
// }

}
