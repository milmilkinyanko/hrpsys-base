// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  RefZMPGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef REFZMPGENERATOR_H
#define REFZMPGENERATOR_H

#include <vector>
#include <memory>
#include <hrpUtil/EigenTypes.h>
#include "FootStepType.h"

namespace hrp
{

class refZMPGenerator
{
  private:
    std::vector<hrp::Vector3> refzmp_list;

    hrp::Vector3 calcRefZMPUsingConstraint(const constraint_count_pairs& contact_points, const double dt, const size_t zmp_index);
    hrp::Vector3 calcRefZMPUsingConstraintList(const std::vector<constraint_count_pairs>& contact_points,
                                               const size_t count);
  public:
    refZMPGenerator() {}
    virtual ~refZMPGenerator() {}

    const std::vector<hrp::Vector3>& getRefZMPList() const { return refzmp_list; }

    void setRefZMPListUsingConstraintList(const std::vector<constraint_count_pairs>& contact_list,
                                          const size_t list_size,
                                          const size_t start_index = 0);
};

}

#endif // REFZMPGENERATOR_H
