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
#include <hrpUtil/EigenTypes.h>
#include "LinkConstraint.h"
#include "interpolator.h"

namespace hrp {

class RefZMPGenerator
{
  private:
    double dt;
    std::deque<hrp::Vector3> refzmp_list;
    std::unique_ptr<interpolator> zmp_interpolator;
    // size_t next_zmp_count;

    // hrp::Vector3 start_zmp; // TODO: 必要かどうか

    hrp::Vector3 calcRefZMP(const ConstraintsWithCount& constraints) const;
    void calcZMPInterpolationGoal(const std::vector<ConstraintsWithCount>& constraints_list,
                                  const size_t cur_constraint_idx,
                                  const size_t cur_count,
                                  const double last_interpolation_time = 2.0);
  public:
    RefZMPGenerator(const double dt, const ConstraintsWithCount& constraints);

    const std::deque<hrp::Vector3>& getRefZMPList() const { return refzmp_list; }

    void setInterpolationAlgorithm(const interpolator::interpolation_mode i_mode)
    {
        zmp_interpolator->setInterpolationMode(i_mode);
    }
    void setRefZMPList(const std::vector<ConstraintsWithCount>& constraints_list,
                       const size_t list_size,
                       const size_t start_index = 0);
    void popAndPushRefZMP(const std::vector<ConstraintsWithCount>& constraints_list,
                          const size_t count);
};

}

#endif // REFZMPGENERATOR_H
