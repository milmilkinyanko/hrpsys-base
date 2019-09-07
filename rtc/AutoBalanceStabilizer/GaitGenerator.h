// -*- C++ -*-

/**
 * @file  GaitGenerator.h
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

namespace hrp {

class GaitGenerator
{
  private:
    hrp::BodyPtr m_robot;
    Eigen::AffineCompact3d root_coord; // m_robot とどっちか？

    // size_t count = 0;
    // double dt;

    // Instance
    std::vector<ConstraintsWithCount> constraints_list;
    std::vector<LimbTrajectoryGenerator> limb_traj_gens;
    RefZMPGenerator zmp_generator;
    COGTrajectoryGenerator cog_generator;

    // -- COGTrajectoryGenerator --
    size_t preview_window = 800;
    // -- COGTrajectoryGenerator --

  public:
    GaitGenerator(hrp::BodyPtr _robot, const double _dt);
    GaitGenerator(hrp::BodyPtr _robot, const double _dt,
                  const std::vector<LinkConstraint>& init_constraints);

    void calcCogAndLimbTrajectory();
    void addLinkConstraint(); // TODO

    // -- COGTrajectoryGenerator --
    const hrp::Vector3& getCog()    const { return cog_generator.getCog(); }
    const hrp::Vector3& getCogVel() const { return cog_generator.getCogVel(); }
    const hrp::Vector3& getCogAcc() const { return cog_generator.getCogAcc(); }
    void setCogCalculationType(COGTrajectoryGenerator::CogCalculationType type) { cog_generator.setCogCalculationType(type); }
    // -- COGTrajectoryGenerator --

    // gopos: 接触のCycleを記述したい
    void goPos(const rats::coordinates& target, const size_t one_step_count,
               const double max_step_length, const double max_rotate_angle,
               const std::vector<std::vector<int>>& support_link_cycle,
               const std::vector<std::vector<int>>& swing_link_cycle);
};

}

#endif // GAITGENERATORABS_H
