// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  GaitGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include "GaitGenerator.h"
#include "Utility.h"

namespace hrp {

GaitGenerator::GaitGenerator(hrp::BodyPtr _robot, const double _dt,
                             std::vector<LinkConstraint>&& init_constraints,
                             const double preview_time = 1.6) :
    preview_window(static_cast<size_t>(std::round(preview_time / _dt)))
{
    constraints_list.resize(1);
    constraints_list[0].constraints = std::move(init_constraints); // TODO: これでmoveできるかテストする
    constraints_list[0].count = 0;

    zmp_generator.setRefZMPListUsingConstraintList(constraints_list, preview_window);
}

void GaitGenerator::calcCogAndLimbTrajectory(const size_t cur_count, const double dt)
{
    zmp_generator.popAndPushRefZMPUsingConstraintList(constraints_list, cur_count + preview_window - 1); // TODO: 1番最初にバグるかも．要確認
    cog_generator.calcCogFromZMP(zmp_generator.getRefZMPList());
    limb_traj_gens.calcTrajectory(count, dt);
}

// TODO: foot stepの生成はm_robotを持っている上位に移すべきか
void GaitGenerator::addNextFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                                const size_t swing_start_count, const double step_time, const double dt,
                                                const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                                const int support_id, const int swing_id)
{
    const ConstraintsWithCount& cur_constraints = getCurrentConstraints(count);
    const int support_idx = cur_constraints.getConstraintIndexFromLinkId(support_id);
    const int swing_idx   = cur_constraints.getConstraintIndexFromLinkId(swing_id);

    if (support_idx == -1 || swing_idx == -1) {
        std::cerr << "Error" << std::endl; // TODO: error message
        return;
    }

    LinkConstraint support = cur_constraints.constraints[support_idx];
    LinkConstraint swing   = cur_constraints.constraints[swing_idx];

    support.setConstraintType(LinkConstraint::FIX);
    swing.setConstraintType(LinkConstraint::FLOAT);

    {
        ConstraintsWithCount swing_phase_constraints;
        swing_phase_constraints.constraints.reserve(2); // TODO: 2足歩行前提
        swing_phase_constraints.push_back(support);
        swing_phase_constraints.push_back(swing);
        swing_phase_constraints.start_count = swing_start_count;

        constraints_list.push_back(std::move(swing_phase_constraints));
    }

    LinkConstraint landing(std::move(swing));
    landing.setConstraintType(LinkConstraint::FIX);

    const double stride_x = vel_x / step_time;
    const double stride_y = vel_y / step_time + landing.targetPos[1]; // TODO: 座標変換 (transform(landing.targetPos))

    // TODO: stride limitation

    landing.targetPos() = support.targetPos() + hrp::Vector3(stride_x, stride_y, 0); // TODO: supportのlocal座標で+してworldに

    {
        ConstraintsWithCount landing_phase_constraints;
        landing_phase_constraints.constraints.reserve(2); // TODO: 2足歩行前提
        landing_phase_constraints.push_back(support);
        landing_phase_constraints.push_back(landing);
        landing_phase_constraints.start_count = swing_start_count + static_cast<size_t>(step_time / dt);

        constraints_list.push_back(std::move(landing_phase_constraints));
    }
}

// Rot は world base rotを使う ?
void GaitGenerator::goPosBiped(const Eigen::AffineCompact3d& target, const size_t one_step_count,
                               const double max_step_length, const double max_rotate_angle /*[rad]*/,
                               const std::vector<int>& support_link_cycle,
                               const std::vector<int>& swing_link_cycle)
{
    const ConstraintsWithCount& cur_constraints = getCurrentConstraints(count);

    const hrp::Vector3 init_cop = cur_constraints.calcCOPFromConstraints();
    const double distance = (init_cop - target.translation()).head<2>().norm();
    const hrp::Matrix33 diff_rot = cur_constraints.calcCOPRotationFromConstraints().transpose() * target.linear();
    const double diff_yaw = rpyFromRot(diff_rot)(2);

    const size_t one_cycle_len = support_link_cycle.size();
    const size_t translation_steps = static_cast<size_t>(std::ceil(distance / max_step_length));
    size_t rotation_steps = static_cast<size_t>(std::ceil(diff_yaw / max_rotate_angle));
    rotation_steps += rotation_steps % one_cycle_len;
    // size_t num_steps = std::max(translation_steps, rotation_steps);
    num_steps = translation_steps; // TODO: tmp straight
    num_steps += num_steps % one_cycle_len;
    const size_t num_cycle = num_steps / one_cycle_len;

    constraints_list.resize(num_steps + 2);
    constraints_list[0] = cur_constraints;

    const hrp::Vector3 one_translation_step = (init_cop - target.translation()) / translation_steps;

    // std::unordered_map<int, Eigen::AffineCompact3d> one_rotation_steps; // 自転, weightが左右異なる場合，左右で違う行列となる
    // for (const LinkConstraint& constraint : cur_constraints.constraints) {
    //     // TODO: 1歩の回転の計算
    // }

    for (size_t i = 0; i < num_cycle; ++i) {
        for (size_t j = 0; j < one_cycle_len; ++j) {
            constraints_list[i + j + 1] = constraints_list[i + j];
            ConstraintsWithCount& next_constraints = constraints_list[i + j + 1];

            const int support_idx = cur_constraints.getConstraintIndexFromLinkId(support_id);
            const int swing_idx   = cur_constraints.getConstraintIndexFromLinkId(swing_id);

            if (support_idx == -1 || swing_idx == -1) {
                std::cerr << "Error" << std::endl; // TODO: error message
                return;
            }

            LinkConstraint& support = cur_constraints.constraints[support_idx];
            LinkConstraint& swing   = cur_constraints.constraints[swing_idx];

            // LinkConstraint& support = next_constraints.getConstraintsFromLinkIds(support_link_cycle[j])[0];
            // LinkConstraint& swing = next_constraints.getConstraintsFromLinkIds(swing_link_cycle[j])[0];

            swing.targetCoord().translation() = support.targetCoord().translation() + one_translation_step;
            // swing.targetCoord().linear() = support.targetCoord().linear(); // 1cycleごとに両足が揃うのか，片方の足を追い越して進んでいくのか
        }
    }
}

void GaitGenerator::goPos(const rats::coordinates& target, const size_t one_step_count,
                          const double max_step_length, const double max_rotate_angle,
                          const std::vector<std::vector<int>>& support_link_cycle,
                          const std::vector<std::vector<int>>& swing_link_cycle)
{
    getCurrentConstraints().calcCOPFromConstraints();
}

}
