// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.cpp
 * @brief
 * @date  $Date$
 */

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

    zmp_interpolator = std::make_unique<interpolator>(3, _dt, interpolator::LINEAR);
    zmp_interpolator->setName("RefZMPGenerator zmp_interpolator");
    zmp_interpolator->set(init_zmp.data());
}

hrp::Vector3 RefZMPGenerator::calcRefZMP(const ConstraintsWithCount& constraints) const
{
    return constraints.calcCOPFromConstraints();
}

void RefZMPGenerator::calcZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                               const size_t start_constraint_idx,
                                               const size_t cur_count,
                                               const size_t start_count,
                                               const double final_interpolation_time)

{
    hrp::Vector3 goal_zmp = refzmp_list[start_count - cur_count]; // Set current zmp as goal
    zmp_interpolator->set(goal_zmp.data());

    size_t goal_count = start_count;

    if (start_constraint_idx + 1 < constraints_list.size()) {
        const hrp::Vector3 next_zmp = calcRefZMP(constraints_list[start_constraint_idx + 1]);

        if (isInsideConvexHull(constraints_list[start_constraint_idx].calcContactConvexHullForAllConstraints(),
                               next_zmp.head<2>())) {
            goal_zmp = next_zmp;
            goal_count = constraints_list[start_constraint_idx + 1].start_count;
        }
    } else { // Final constraint
        goal_zmp = calcRefZMP(constraints_list[start_constraint_idx]);
        goal_count = start_count + static_cast<size_t>(final_interpolation_time / dt);
    }

    // std::cerr << "start_zmp:  " << refzmp_list[start_count - cur_count].transpose() << ", diff: " << start_count - cur_count << std::endl;
    // std::cerr << "goal_zmp: " << goal_zmp.transpose() << ", diff_count: " << goal_count - start_count << std::endl;
    zmp_interpolator->setGoal(goal_zmp.data(), nullptr, (goal_count - start_count) * dt, true);
}

void RefZMPGenerator::setRefZMPList(const std::vector<ConstraintsWithCount>& constraints_list,
                                    const size_t cur_count, const size_t zmp_start_index)
{
    const size_t cwc_size = constraints_list.size();
    const size_t max_count = cur_count + refzmp_list.size();
    // TODO: 現在のindexを上書くと次に呼ばれるforwardで消えてしまうので，1個次のindexから入れていく．とてもわかりづらい
    size_t count = cur_count + zmp_start_index + 1;
    size_t zmp_index = zmp_start_index + 1;

    // TODO: flight phase
    for (size_t constraint_index = getConstraintIndexFromCount(constraints_list, count - 1);
         constraint_index < cwc_size && count < max_count;
         ++constraint_index) {
        std::cerr << "zmp const index: " << constraint_index << std::endl;
        // TODO: interpolatorの使い方がおかしい？前に設定したgoalから新しいgoalへの補間になっている？
        calcZMPInterpolationGoal(constraints_list, constraint_index, cur_count, count - 1);
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
        calcZMPInterpolationGoal(constraints_list, cur_index, cur_count, future_count);
    }

    hrp::Vector3 next_zmp;
    zmp_interpolator->get(next_zmp.data(), true);
    refzmp_list.pop_front();
    refzmp_list.push_back(next_zmp);
}

}
