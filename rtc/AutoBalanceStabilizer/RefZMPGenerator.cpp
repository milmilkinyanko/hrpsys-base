// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <iostream>
#include <iterator>
#include "PlaneGeometry.h"
#include "RefZMPGenerator.h"

namespace hrp {

RefZMPGenerator::RefZMPGenerator(const double _dt, const size_t list_size,
                                 const ConstraintsWithCount& init_constraints)
    : dt(_dt)
{
    const hrp::Vector3 init_zmp = calcRefZMP(init_constraints);
    refzmp_list.resize(list_size, init_zmp);

    std::cerr << "init_zmp: " << init_zmp.transpose() << std::endl;

    zmp_interpolator = std::make_unique<interpolator>(3, _dt, interpolator::LINEAR);
    zmp_interpolator->setName("RefZMPGenerator zmp_interpolator");
    zmp_interpolator->set(init_zmp.data());
}

hrp::Vector3 RefZMPGenerator::calcRefZMP(const ConstraintsWithCount& constraints) const
{
    return constraints.calcCOPFromConstraints();
}

std::pair<hrp::Vector3, size_t> RefZMPGenerator::calcZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                                                          const hrp::Vector3& start_zmp,
                                                                          const size_t start_constraint_idx,
                                                                          const size_t start_count,
                                                                          const double final_interpolation_time)

{
    hrp::Vector3 goal_zmp = start_zmp;
    size_t goal_count = start_count;

    if (start_constraint_idx + 1 < constraints_list.size()) {
        const hrp::Vector3 next_zmp = constraints_list[start_constraint_idx + 1].calcStartCOPFromConstraints();
        goal_count = constraints_list[start_constraint_idx + 1].start_count;

        if (isInsideConvexHull(constraints_list[start_constraint_idx].calcContactConvexHullForAllConstraints(),
                               next_zmp.head<2>())) {
            goal_zmp = next_zmp;
        }
    } else { // Final constraint
        goal_zmp = constraints_list[start_constraint_idx].calcCOPFromConstraints();
        goal_count = start_count + static_cast<size_t>(final_interpolation_time / dt);
        std::cerr << "goal_zmp: " << goal_zmp.transpose() << ", goal_count: " << goal_count << ", start_zmp: " << start_zmp.transpose() << ", start_count: " << start_count << std::endl;
    }

    return std::make_pair(goal_zmp, goal_count);
}

std::vector<std::pair<hrp::Vector3, size_t>> RefZMPGenerator::calcZMPGoalsFromConstraints(const std::vector<ConstraintsWithCount>& constraints_list,
                                                                                          const size_t start_constraint_idx,
                                                                                          const hrp::Vector3 start_zmp,
                                                                                          const size_t start_count,
                                                                                          const double final_interpolation_time)
{
    const int landing_idx = getNextStableConstraints(constraints_list, start_constraint_idx);
    if (landing_idx == -1) {
        std::cerr << "Cannot find next stable constraints and current constraints are also unstable. Something wrong!!" << std::endl;
        return std::vector<std::pair<hrp::Vector3, size_t>>();
    }

    std::vector<std::pair<hrp::Vector3, size_t>> zmp_goals;
    zmp_goals.reserve(landing_idx - start_constraint_idx + 2);
    // TODO: 一番最後のconstraints
    // zmp_goals.push_back(std::make_pair(constraints_list[start_constraint_idx].calcStartCOPFromConstraints(), constraints_list[start_constraint_idx].start_count));
    zmp_goals.push_back(std::make_pair(start_zmp, start_count));

    for (size_t idx = start_constraint_idx; idx < landing_idx + 1; ++idx) {
        const std::pair<hrp::Vector3, size_t> zmp_goal = calcZMPInterpolationGoal(constraints_list, zmp_goals.back().first, idx, constraints_list[idx].start_count, final_interpolation_time);
        zmp_goals.push_back(zmp_goal);
        // if (zmp_goal.second > constraints_list[idx].start_count) zmp_goals.push_back(zmp_goal);
    }

    // zmp_goals.push_back(std::make_pair(constraints_list[landing_idx].calcCOPFromConstraints(), constraints_list[landing_idx].start_count));
    std::cerr << "calc zmp goals: final: " << zmp_goals.back().first.transpose() << std::endl;
    return zmp_goals;
}

void RefZMPGenerator::calcAndSetZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                                     const size_t start_constraint_idx,
                                                     const size_t cur_count,
                                                     const size_t start_count,
                                                     const double final_interpolation_time)
{
    zmp_interpolator->set(refzmp_list[start_count - cur_count].data());
    std::pair<hrp::Vector3, size_t> goals = calcZMPInterpolationGoal(constraints_list, refzmp_list[start_count - cur_count], start_constraint_idx, start_count, final_interpolation_time);
    zmp_interpolator->setGoal(goals.first.data(), nullptr, (goals.second - start_count) * dt, true);
}

void RefZMPGenerator::setRefZMPList(const std::vector<ConstraintsWithCount>& constraints_list,
                                    const size_t cur_count, const size_t zmp_start_index)
{
    const size_t cwc_size = constraints_list.size();
    const size_t max_count = cur_count + refzmp_list.size();
    // TODO: 現在のindexを上書くと次に呼ばれるforwardで消えてしまうので，1個次のindexから入れていく．とてもわかりづらい
    size_t count = cur_count + zmp_start_index + 1;
    size_t zmp_index = zmp_start_index + 1;

    // TODO: flight phase後に補間というよりは一気に移動させる
    for (size_t constraint_index = getConstraintIndexFromCount(constraints_list, count - 1);
         constraint_index < cwc_size && count < max_count;
         ++constraint_index) {

        calcAndSetZMPInterpolationGoal(constraints_list, constraint_index, cur_count, count - 1);
        const size_t cur_max_count = (constraint_index < cwc_size - 1) ? std::min(constraints_list[constraint_index + 1].start_count + 1, max_count) : max_count;

        for (; count < cur_max_count; ++zmp_index, ++count) {
            zmp_interpolator->get(refzmp_list[zmp_index].data(), true);
        }
    }
}

void RefZMPGenerator::popAndPushRefZMP(const std::vector<ConstraintsWithCount>& constraints_list,
                                       const size_t cur_count)
{
    const size_t future_count = cur_count + refzmp_list.size() - 1;

    // TODO: ZMPを補間するのではなく、Constraintのweightを補間する方が自然で楽? ZMP制御ではなくなった時も応用が効きそうだし、力分配もそのまま使えそう <- zmpを支持領域で打ち切るのが困難?
    const size_t cur_index = getConstraintIndexFromCount(constraints_list, future_count);
    if (constraints_list[cur_index].start_count == future_count) { // 歩き始めにZMPの補間がされなくなる
        std::cerr << "zmp const index: " << cur_index << std::endl;
        calcAndSetZMPInterpolationGoal(constraints_list, cur_index, cur_count, future_count);
    }

    hrp::Vector3 next_zmp;
    zmp_interpolator->get(next_zmp.data(), refzmp_vel.data(), true);
    refzmp_list.pop_front();
    refzmp_list.push_back(next_zmp);
}

}
