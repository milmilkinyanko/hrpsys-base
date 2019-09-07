// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <iterator>
#include "RefZMPGenerator.h"

namespace hrp {

// Push or just calc
// TODO: move to LinkConstraint.h
hrp::Vector3 RefZMPGenerator::calcRefZMPUsingConstraintList(const std::vector<ConstraintsWithCount>& constraints_list,
                                                            const size_t count)
{
    const size_t constraint_idx = getConstraintIndexFromCount(constraints_list, count);
    return constraints_list[constraint_idx].calcCOPFromConstraints();
}

// No interpolation
void RefZMPGenerator::setRefZMPListUsingConstraintList(const std::vector<ConstraintsWithCount>& constraints_list,
                                                       const size_t list_size,
                                                       const size_t start_index)
{
    refzmp_list.resize(list_size);

    size_t count = constraints_list[0].start_count;
    size_t constraint_index = 0;
    size_t zmp_index = start_index;
    const size_t constraint_size = constraints_list.size();
    const size_t max_count = count + list_size;
    count += start_index;

    // TODO: zmp offset, flight phase
    for (size_t constraint_index = 0; count < max_count && constraint_index < constraint_size; ++constraint_index) {
        if (count < constraints_list[constraint_index].start_count) continue;

        const hrp::Vector3 zmp_pos = constraints_list[constraint_index].calcCOPFromConstraints();
        const size_t max_current_count = (constraint_index < constraint_size - 1) ? std::min(constraints_list[constraint_index + 1].start_count, max_count) : max_count;
        // TODO: for消せそう
        for (; count < max_current_count; ++zmp_index, ++count) {
            refzmp_list[zmp_index] = zmp_pos;
        }
    }
}

void RefZMPGenerator::popAndPushRefZMPUsingConstraintList(const std::vector<ConstraintsWithCount>& constraints_list,
                                                          const size_t count)
{
    refzmp_list.pop_front();
    refzmp_list.push_back(calcRefZMPUsingConstraintList(constraints_list, count));
}

}
