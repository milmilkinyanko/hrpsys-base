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
    m_robot(boost::make_shared<hrp::Body>(*_robot)),
    root_id(_robot->rootLink()->index),
    preview_window(static_cast<size_t>(std::round(preview_time / _dt)))
{
    const size_t num_limbs = init_constraints.size();

    // Set initial position as target (need to be done calcForwardKinematics)
    // TODO: ここで良いか
    for (auto&& constraint : init_constraints) {
        hrp::Link* link = _robot->link(constraint.getLinkId());
        std::cerr << "baka: " << link->name << " id: " << constraint.getLinkId() << " pos: " << link->p.transpose() << std::endl;
        constraint.targetPos() = link->p + link->R * constraint.getLinkRepresentativePoint(); // TODO: 確認
        constraint.targetRot() = link->R;
    }

    constraints_list.resize(1);
    constraints_list[0].constraints = std::move(init_constraints); // TODO: これでmoveできるかテストする
    constraints_list[0].start_count = 0;

    limb_gens.resize(num_limbs);
    for (size_t i = 0; i < num_limbs; ++i) {
        std::cerr << "i: " << i << " / " << num_limbs << std::endl; // TODO: debug
        limb_gens[i].setPos(static_cast<hrp::Vector3>(constraints_list[0].constraints[i].targetPos()));
    }

    root_coord.translation() = _robot->rootLink()->p;
    root_coord.linear() = _robot->rootLink()->R;

    zmp_gen = std::make_unique<RefZMPGenerator>(_dt, preview_window, constraints_list[0]);
    cog_gen = std::make_unique<COGTrajectoryGenerator>(_robot->calcCM());

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

    for (auto limb_gen : limb_gens) {
        limb_gen.calcTrajectory(cur_count, dt);
    }

    for (size_t i = 0; i < limb_gens.size(); ++i) {
        constraints_list[cur_const_idx].constraints[i].targetPos() = limb_gens[i].getPos();
        // constraints_list[cur_const_idx].constraints[i].targetRot() = limb_gens[i].getRot(); // TODO
    }

    // TODO: rootlinkの計算方法，rotへの対応
    //       constraintsにrootlinkを含めて良いかも確認
    // const int root_idx = constraints_list[cur_const_idx].getConstraintIndexFromLinkId(root_id);
    // constraints_list[cur_const_idx].constraints[root_idx].targetPos() = cog_gen->getCog();

    return;
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

    LinkConstraint support = constraints_list[cur_idx].constraints[support_idx];
    LinkConstraint swing   = constraints_list[cur_idx].constraints[swing_idx];

    support.setConstraintType(LinkConstraint::FIX);
    swing.setConstraintType(LinkConstraint::FLOAT);

    {
        ConstraintsWithCount swing_phase_constraints;
        swing_phase_constraints.constraints.reserve(2); // TODO: 2足歩行前提
        swing_phase_constraints.constraints.push_back(support);
        swing_phase_constraints.constraints.push_back(swing);
        swing_phase_constraints.start_count = swing_start_count;

        constraints_list.push_back(std::move(swing_phase_constraints));
    }

    LinkConstraint landing(std::move(swing));
    landing.setConstraintType(LinkConstraint::FIX);

    {
        // Support limb coordinate
        double stride_x = vel_x / step_time;
        double stride_y = vel_y / step_time + (support.targetRot().inverse() * (landing.targetPos() - support.targetPos()))[1];
        double yaw_angle = vel_yaw / step_time;

        // TODO: stride limitation

        Eigen::Isometry3d trans_sup_local;
        trans_sup_local.translation() = Eigen::Vector3d(stride_x, stride_y, 0);
        trans_sup_local.linear() = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        landing.targetCoord() = (support.targetCoord() * trans_sup_local) * support.targetCoord(); // TODO: evalいらないか確認, 合ってるか確認
    }

    {
        ConstraintsWithCount landing_phase_constraints;
        landing_phase_constraints.constraints.reserve(2); // TODO: 2足歩行前提
        landing_phase_constraints.constraints.push_back(support);
        landing_phase_constraints.constraints.push_back(landing);
        landing_phase_constraints.start_count = swing_start_count + static_cast<size_t>(step_time / dt);

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
