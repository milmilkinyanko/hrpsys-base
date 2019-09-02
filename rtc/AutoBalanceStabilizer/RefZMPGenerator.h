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

namespace hrp {

class RefZMPGenerator
{
  private:
    std::deque<hrp::Vector3> refzmp_list;

    hrp::Vector3 calcRefZMPUsingConstraintList(const std::vector<ConstraintsWithCount>& constraints_list,
                                               const size_t count);
  public:
    RefZMPGenerator() {}

    const std::deque<hrp::Vector3>& getRefZMPList() const { return refzmp_list; }

    void setRefZMPListUsingConstraintList(const std::vector<ConstraintsWithCount>& constraints_list,
                                          const size_t list_size,
                                          const size_t start_index = 0);
    void popAndPushRefZMPUsingConstraintList(const std::vector<ConstraintsWithCount>& constraints_list,
                                             const size_t count);
};

}

#endif // REFZMPGENERATOR_H
