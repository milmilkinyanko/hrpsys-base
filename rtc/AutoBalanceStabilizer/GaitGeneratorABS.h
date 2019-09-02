// -*- C++ -*-

/**
 * @file  GaitGeneratorABS.h
 * @brief
 * @date  $Date$
 */

#ifndef GAITGENERATORABS_H
#define GAITGENERATORABS_H

#include <vector>
#include <deque>
#include "../ImpedanceController/RatsMatrix.h"
#include "LinkConstraint.h"
#include "RefZMPGenerator.h"

namespace hrp
{

class GaitGenerator
{
  private:
    // Instance
    refZMPGenerator zmp_generator;
    refCOGGenerator cog_generator;
    footTrajectoryGenerator foot_trj_generator;

    std::vector<step_node> steps;
    contact_pair_list contact_points;

  public:
    GaitGenerator() {};
    virtual ~GaitGenerator() {};
};

}

#endif // GAITGENERATORABS_H
