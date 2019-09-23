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

#include "Utility.h"
#include "LinkConstraint.h"
#include "RefZMPGenerator.h"
#include "LimbTrajectoryGenerator.h"
#include "COGTrajectoryGenerator.h"

namespace hrp {

class GaitGenerator
{
  private:
    const size_t root_id = 0;
    Eigen::Isometry3d root_coord = Eigen::Isometry3d::Identity();

    size_t cur_const_idx = 0; // To reduce calculation time

    // Gait parameter
    double default_single_support_time = 1.0;
    double default_double_support_time = 0.25;
    double default_step_height = 0.10;
    double max_stride = 1.5;
    double max_rot_angle = deg2rad(90);
    std::vector<int> default_support_cycle; // TODO: std::vector<std::vector<int>>
    std::vector<int> default_swing_cycle;
    size_t current_cycle = 0;

    // Velocity mode parameter
    bool is_velocity_mode = false; // TODO: enum?
    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;

    // Instance
    std::vector<ConstraintsWithCount> constraints_list;
    std::vector<LimbTrajectoryGenerator> limb_gens;
    std::unique_ptr<RefZMPGenerator> zmp_gen;
    std::unique_ptr<COGTrajectoryGenerator> cog_gen;
    // TODO: STも持っても良いかも (AutoBalancerを上ではなくする)

    hrp::Vector3 prev_ref_cog = hrp::Vector3::Zero();
    // TODO: 使い方をよく考える
    //       default
    std::vector<std::vector<int>> support_limb_cycle;
    std::vector<std::vector<int>> swing_limb_cycle;

    // -- COGTrajectoryGenerator --
    size_t preview_window = 800;
    // -- COGTrajectoryGenerator --

    // -- LimbTrajectoryGenerator --
    LimbTrajectoryGenerator::TrajectoryType default_traj_type = LimbTrajectoryGenerator::CYCLOIDDELAY;
    // -- LimbTrajectoryGenerator --

    void addFootStepVelocityMode(const size_t cur_count);

  public:
    GaitGenerator(const hrp::BodyPtr& _robot,
                  const double _dt,
                  std::vector<LinkConstraint>&& init_constraints,
                  const double preview_time = 1.6);

    void setDefaultStepHeight(const double _height) { default_step_height = _height; }

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

    // Todo: Private ?
    hrp::Vector3 calcReferenceCOPFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const;
    hrp::Matrix33 calcReferenceCOPRotFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const;
    void adjustCOPCoordToTarget(const hrp::BodyPtr& _robot, const size_t count);

    // -- RefZMPGenerator --
    void setRefZMPList(const size_t count, const size_t start_index = 0)
    {
        zmp_gen->setRefZMPList(constraints_list, count, start_index);
    }
    const hrp::Vector3& getCurrentRefZMP() const { return zmp_gen->getCurrentRefZMP(); }
    // -- RefZMPGenerator --

    // -- COGTrajectoryGenerator --
    void resetCOGTrajectoryGenerator(const hrp::Vector3& init_cog, const double dt)
    {
        cog_gen.reset(new COGTrajectoryGenerator(init_cog));
        cog_gen->initPreviewController(dt, zmp_gen->getCurrentRefZMP());
    }
    const hrp::Vector3& getCog()    const { return cog_gen->getCog(); }
    const hrp::Vector3& getCogVel() const { return cog_gen->getCogVel(); }
    const hrp::Vector3& getCogAcc() const { return cog_gen->getCogAcc(); }
    void setCogCalculationType(COGTrajectoryGenerator::CogCalculationType type)
    {
        cog_gen->setCogCalculationType(type);
    }
    // -- COGTrajectoryGenerator --

    void startVelocityMode(const double _vel_x, const double _vel_y, const double _vel_yaw)
    {
        vel_x = _vel_x;
        vel_y = _vel_y;
        vel_yaw = _vel_yaw;
        is_velocity_mode = true;
    }

    void stopVelocityMode()
    {
        // TODO: addNextFootStepFromVelocity(0, 0, 0, ...) // ２脚ならこれでそろうが // cycleが終わるまでやる？
        is_velocity_mode = false;
    }

    // TODO: for debug
    void addCurrentConstraintForDummy(const size_t start_count)
    {
        const size_t cur_idx = getConstraintIndexFromCount(constraints_list, start_count);
        constraints_list.reserve(constraints_list.size() + 1);

        ConstraintsWithCount dummy_constraints(constraints_list[cur_idx]);
        dummy_constraints.start_count = start_count;
        constraints_list.push_back(std::move(dummy_constraints));
    }

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
