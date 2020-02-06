// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef REFZMPGENERATOR_H
#define REFZMPGENERATOR_H

#include <vector>
#include <deque>
#include <memory>
#include <utility>
#include <hrpUtil/EigenTypes.h>
#include "LinkConstraint.h"
#include "interpolator.h"

namespace hrp {

class RefZMPGenerator
{
  private:
    double dt;
    std::deque<hrp::Vector3> refzmp_list;
    hrp::Vector3 refzmp_vel = hrp::Vector3::Zero();
    std::unique_ptr<interpolator> zmp_interpolator;

    hrp::Vector3 calcRefZMP(const ConstraintsWithCount& constraints) const;
    std::pair<hrp::Vector3, size_t> calcZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                                             const hrp::Vector3& start_zmp,
                                                             const size_t start_constraint_idx,
                                                             const size_t start_count,
                                                             const double final_interpolation_time = 1.6);
    void calcAndSetZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                        const size_t start_constraint_idx,
                                        const size_t cur_count,
                                        const size_t start_count,
                                        const double final_interpolation_time = .6);
  public:
    RefZMPGenerator(const double dt, const size_t list_size,
                    const ConstraintsWithCount& init_constraints);

    const hrp::Vector3& getCurrentRefZMP() const { return refzmp_list.front(); }
    const std::deque<hrp::Vector3>& getRefZMPList() const { return refzmp_list; }
    const hrp::Vector3& getRefZMPVel() const { return refzmp_vel; }

    void setInterpolationAlgorithm(const interpolator::interpolation_mode i_mode)
    {
        zmp_interpolator->setInterpolationMode(i_mode);
    }
    void setRefZMPList(const std::vector<ConstraintsWithCount>& constraints_list,
                       const size_t cur_count, const size_t zmp_start_index = 0);
    void popAndPushRefZMP(const std::vector<ConstraintsWithCount>& constraints_list,
                          const size_t cur_count);
    std::vector<std::pair<hrp::Vector3, size_t>> calcZMPGoalsFromConstraints(const std::vector<ConstraintsWithCount>& constraints_list,
                                                                             const size_t start_constraint_idx,
                                                                             const hrp::Vector3 start_zmp,
                                                                             const size_t start_count,
                                                                             const double final_interpolation_time = 1.6);
};

}

#endif // REFZMPGENERATOR_H
