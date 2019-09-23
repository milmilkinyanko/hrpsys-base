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

// 1つ次のZMPに補間
// その際，支持領域の関係で補間できない時は，補間しない ( or, 支持領域で打ち切る)
void RefZMPGenerator::calcZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                               const size_t cur_constraint_idx,
                                               const size_t cur_count,
                                               const double last_interpolation_time)

{
    hrp::Vector3 goal_zmp;
    zmp_interpolator->get(goal_zmp.data(), false); // Set current zmp as goal_zmp

    size_t goal_count = cur_count;

    if (cur_constraint_idx + 1 < constraints_list.size()) {
        const hrp::Vector3 next_zmp = calcRefZMP(constraints_list[cur_constraint_idx + 1]);

        if (isInsideConvexHull(constraints_list[cur_constraint_idx].calcContactConvexHullForAllConstraints(),
                               next_zmp.head<2>())) {
            goal_zmp = next_zmp;
            goal_count = constraints_list[cur_constraint_idx + 1].start_count;
        }
    } else { // Last constraint
        goal_zmp = calcRefZMP(constraints_list[cur_constraint_idx]);
        goal_count = cur_count + static_cast<size_t>(last_interpolation_time / dt);
    }

    zmp_interpolator->setGoal(goal_zmp.data(), (goal_count - cur_count) * dt, true);
}

// 両脚支持期間は線形補間しとく
// 次のZMPを見て，現在のZMPから補間するようにする
// ZMPは支持領域で打ち切る
void RefZMPGenerator::setRefZMPList(const std::vector<ConstraintsWithCount>& constraints_list,
                                    const size_t cur_count, const size_t start_index)
{
    const size_t constraint_size = constraints_list.size();
    const size_t max_count = cur_count + refzmp_list.size();
    size_t count = cur_count + start_index;
    size_t zmp_index = start_index;
    size_t constraint_index = getConstraintIndexFromCount(constraints_list, count);

    // TODO: flight phase
    for (size_t constraint_index = 0; count < max_count && constraint_index < constraint_size; ++constraint_index) {
        calcZMPInterpolationGoal(constraints_list, constraint_index, count);
        const size_t cur_max_count = (constraint_index < constraint_size - 1) ? std::min(constraints_list[constraint_index + 1].start_count, max_count) : max_count;

        for (; count < cur_max_count; ++zmp_index, ++count) {
            zmp_interpolator->get(refzmp_list[zmp_index].data(), true);
        }
    }
}

void RefZMPGenerator::popAndPushRefZMP(const std::vector<ConstraintsWithCount>& constraints_list,
                                       const size_t cur_count)
{
    refzmp_list.pop_front();
    const size_t future_count = cur_count + refzmp_list.size() - 1;

    // TODO: ZMPを補間するのではなく、Constraintのweightを補間する方が自然で楽? ZMP制御ではなくなった時も応用が効きそうだし、力分配もそのまま使えそう
    const size_t cur_index = getConstraintIndexFromCount(constraints_list, future_count);
    if (constraints_list[cur_index].start_count == future_count) { // 歩き始めにZMPの補間がされなくなる
        calcZMPInterpolationGoal(constraints_list, cur_index, future_count);
    }

    hrp::Vector3 next_zmp;
    zmp_interpolator->get(next_zmp.data(), true);
    refzmp_list.push_back(next_zmp);
}

}
