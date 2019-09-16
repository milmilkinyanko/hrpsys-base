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
#include <memory>

#include <hrpModel/Link.h>
#include <hrpModel/Body.h>

#include "LinkConstraint.h"
#include "RefZMPGenerator.h"
#include "LimbTrajectoryGenerator.h"
#include "COGTrajectoryGenerator.h"

namespace hrp {

class GaitGenerator
{
  private:
    hrp::BodyPtr m_robot;
    const size_t root_id = 0;
    Eigen::Isometry3d root_coord = Eigen::Isometry3d::Identity(); // m_robot とどっちか？

    // size_t count = 0;
    // double dt;

    size_t cur_const_idx = 0; // To reduce calculation time

    // Instance
    std::vector<ConstraintsWithCount> constraints_list;
    std::vector<LimbTrajectoryGenerator> limb_gens;
    std::unique_ptr<RefZMPGenerator> zmp_gen;
    std::unique_ptr<COGTrajectoryGenerator> cog_gen;
    // TODO: STも持っても良いかも (AutoBalancerを上ではなくする)

    // TODO: 使い方をよく考える
    //       default
    std::vector<std::vector<int>> support_limb_cycle;
    std::vector<std::vector<int>> swing_limb_cycle;

    // -- COGTrajectoryGenerator --
    size_t preview_window = 800;
    // -- COGTrajectoryGenerator --

    // -- LimbTrajectoryGenerator --
    LimbTrajectoryGenerator::TrajectoryType default_traj_type = LimbTrajectoryGenerator::CYCLOIDDELAY;
    double default_step_height = 0.10;
    // -- LimbTrajectoryGenerator --

  public:
    // GaitGenerator(hrp::BodyPtr _robot, const double _dt, const double preview_time = 1.6);
    GaitGenerator(const hrp::BodyPtr& _robot,
                  const double _dt,
                  std::vector<LinkConstraint>&& init_constraints,
                  const double preview_time = 1.6);

    void forwardTimeStep(const size_t cur_count);
    void calcCogAndLimbTrajectory(const size_t cur_count, const double dt);
    void addLinkConstraint(); // TODO

    const ConstraintsWithCount& getCurrentConstraints(const size_t count) const
    {
        const size_t index = getConstraintIndexFromCount(constraints_list, count);
        return constraints_list[index];
    }
    const std::vector<ConstraintsWithCount>& getConstraintsList() const { return constraints_list; }

    Eigen::Isometry3d& rootCoord() { return root_coord; }
    const Eigen::Isometry3d& rootCoord() const { return root_coord; }
    Eigen::Isometry3d::TranslationPart rootPos() { return root_coord.translation(); }
    Eigen::Isometry3d::ConstTranslationPart rootPos() const { return root_coord.translation(); }
    Eigen::Isometry3d::LinearPart rootRot() { return root_coord.linear(); }
    Eigen::Isometry3d::ConstLinearPart rootRot() const { return root_coord.linear(); }


    // -- RefZMPGenerator --
    const hrp::Vector3& getCurrentRefZMP() const { return zmp_gen->getCurrentRefZMP(); }
    // -- RefZMPGenerator --

    // -- COGTrajectoryGenerator --
    const hrp::Vector3& getCog()    const { return cog_gen->getCog(); }
    const hrp::Vector3& getCogVel() const { return cog_gen->getCogVel(); }
    const hrp::Vector3& getCogAcc() const { return cog_gen->getCogAcc(); }
    void setCogCalculationType(COGTrajectoryGenerator::CogCalculationType type)
    {
        cog_gen->setCogCalculationType(type);
    }
    // -- COGTrajectoryGenerator --


    void addNextFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                     const size_t swing_start_count, const double step_time, const double dt,
                                     const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                     const int support_id, const int swing_id);

    // gopos: 接触のCycleを記述したい
    // void goPos(const rats::coordinates& target, const size_t one_step_count,
    //            const double max_step_length, const double max_rotate_angle,
    //            const std::vector<std::vector<int>>& support_link_cycle,
    //            const std::vector<std::vector<int>>& swing_link_cycle);
};

}

#endif // GAITGENERATORABS_H
