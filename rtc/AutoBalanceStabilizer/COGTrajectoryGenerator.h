// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef COGTRAJECTORYGENERATOR_H
#define COGTRAJECTORYGENERATOR_H

#include <vector>
#include <memory>
#include <hrpUtil/EigenTypes.h>

namespace hrp
{

class COGTrajectoryGenerator
{
  private:
    hrp::Vector3 cog;
    hrp::Vector3 cog_vel;
    hrp::Vector3 cog_acc;

  public:
    COGTrajectoryGenerator() {}
    virtual ~COGTrajectoryGenerator() {}

    const hrp::Vector3& getCog() const { return cog; }
    const hrp::Vector3& getCogVel() const { return cog_vel; }
    const hrp::Vector3& getCogAcc() const { return cog_acc; }

    void calcCogTrajectory(const std::vector<hrp::Vector3>& refzmp_list);
};

}

#endif // COGTRAJECTORYGENERATOR_H
