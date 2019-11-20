// -*- C++ -*-

/**
 * @file  GaitGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef GAITGENERATORABS_H
#define GAITGENERATORABS_H

#include <vector>
#include <memory>
#include <mutex>

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
    std::mutex& m_mutex; // This is the reference to the mutex of AutoBalanceStabilizer class
    const size_t root_id = 0;
    Eigen::Isometry3d root_coord = Eigen::Isometry3d::Identity();
    size_t loop = 0;

    size_t cur_const_idx = 0; // To reduce calculation time

    /// Gait parameter
    size_t default_single_support_count;
    size_t default_double_support_count;
    size_t default_toe_support_count;  // memo: toeは single supportに含まれる
    size_t default_heel_support_count; // memo: heelはdouble supportに含まれる
    double default_step_height = 0.10; // [m]
    double max_stride = 0.1; // [m]
    double max_rot_angle = deg2rad(10); // [rad]

    // Walking cycle
    std::vector<int> support_limb_cycle; // TODO: std::vector<std::vector<int>>
    std::vector<int> swing_limb_cycle;
    // std::vector<std::vector<int>> support_limb_cycle;
    // std::vector<std::vector<int>> swing_limb_cycle;
    size_t current_cycle = 0;

    // Toe-heel parameter
    bool default_use_toe_heel = false;
    double toe_kick_angle = deg2rad(15); // [rad]
    double heel_contact_angle = deg2rad(15); // [rad]

    // Velocity mode parameter
    bool is_velocity_mode = false; // TODO: enum?
    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;

    /// Instance
    std::vector<ConstraintsWithCount> constraints_list;
    std::vector<LimbTrajectoryGenerator> limb_gens;
    std::unique_ptr<RefZMPGenerator> zmp_gen;
    std::unique_ptr<COGTrajectoryGenerator> cog_gen;
    // TODO: STも持っても良いかも (AutoBalancerを上ではなくする)

    hrp::Vector3 prev_ref_cog = hrp::Vector3::Zero();

    // -- COGTrajectoryGenerator --
    size_t preview_window = 800;
    // -- COGTrajectoryGenerator --

    // -- LimbTrajectoryGenerator --
    LimbTrajectoryGenerator::TrajectoryType default_traj_type = LimbTrajectoryGenerator::RECTANGLE;
    // -- LimbTrajectoryGenerator --

    void addFootStepVelocityMode(const size_t cur_count);

  public:
    GaitGenerator(const hrp::BodyPtr& _robot,
                  std::mutex& _mutex,
                  const double _dt,
                  std::vector<LinkConstraint>&& init_constraints,
                  const double preview_time = 1.6);

    void resetGaitGenerator(const hrp::BodyPtr& _robot,
                            const size_t cur_count,
                            const double _dt);

    void setCurrentLoop(const size_t _loop) { loop = _loop; }

    void forwardTimeStep(const size_t cur_count);
    void calcCogAndLimbTrajectory(const size_t cur_count, const double dt);
    void addLinkConstraint(); // TODO

    void setConstraintsList(const decltype(constraints_list)& _constraints) { constraints_list = _constraints; }
    void setConstraintsList(decltype(constraints_list)&& _constraints) { constraints_list = _constraints; }
    void addConstraintsList(const decltype(constraints_list)& _constraints)
    {
        constraints_list.reserve(constraints_list.size() + _constraints.size());
        for (const auto& constraints : _constraints) {
            constraints_list.push_back(constraints);
        }
    }
    const ConstraintsWithCount& getCurrentConstraints(const size_t count) const
    {
        const size_t index = getConstraintIndexFromCount(constraints_list, count);
        return constraints_list[index];
    }
    const std::vector<ConstraintsWithCount>& getConstraintsList() const { return constraints_list; }
    void modifyConstraintsTarget(const size_t cur_count,
                                 const size_t cwc_idx_from_current,
                                 const size_t modif_const_idx,
                                 const Eigen::Isometry3d& modif_mat,
                                 const double dt);

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
    void setRefZMPList(const size_t cur_count, const size_t start_index = 0)
    {
        zmp_gen->setRefZMPList(constraints_list, cur_count, start_index);
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

    // -- LimbTrajectoryGenerator --
    void modifyLimbViaPoints(const size_t constraint_idx,
                             const Eigen::Isometry3d& new_goal,
                             const size_t cur_count,
                             const size_t new_goal_count);
    // -- LimbTrajectoryGenerator --

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

    std::vector<ConstraintsWithCount>
    calcFootStepConstraints(const ConstraintsWithCount& last_constraints,
                            const std::vector<size_t>& swing_indices,
                            const std::vector<Eigen::Isometry3d>& targets,
                            const size_t swing_start_count,
                            const size_t one_step_count,
                            const bool use_toe_heel = false,
                            const std::vector<size_t>& toe_support_indices = std::vector<size_t>(),
                            const size_t toe_support_count = 0,
                            const size_t heel_support_count = 0);

    void addFootStep(const ConstraintsWithCount& last_constraints,
                     const std::vector<size_t>& swing_indices,
                     const std::vector<Eigen::Isometry3d>& targets,
                     const size_t swing_start_count,
                     const size_t one_step_count,
                     const bool use_toe_heel = false,
                     const std::vector<size_t>& toe_support_indices = std::vector<size_t>(),
                     const size_t toe_support_count = 0,
                     const size_t heel_support_count = 0);

    void addFootStep(const std::vector<int>& link_ids, const std::vector<Eigen::Isometry3d>& targets,
                     const size_t swing_start_count, const size_t one_step_count);

    void addFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                 const size_t swing_start_count, const double step_time, const double dt,
                                 const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                 const int support_id, const int swing_id);


    /* Service below */
    std::vector<int> getConstraintLinkIds();
    void setDefaultSingleSupportTime(const double time, const double dt);
    void setDefaultDoubleSupportTime(const double time, const double dt);
    void setDefaultToeSupportTime(const double time, const double dt);
    void setDefaultHeelSupportTime(const double time, const double dt);
    void setDefaultStepHeight(const double height)   { default_step_height = height; }
    void setMaxStride(const double stride)           { max_stride = stride; }
    void setMaxRotAngle(const double angle_rad)      { max_rot_angle = angle_rad; }

    void setUseToeHeel(const bool use_toe_heel)      { default_use_toe_heel = use_toe_heel; }
    void setToeKickAngle(const double angle_rad)     { toe_kick_angle = angle_rad; }
    void setHeelContactAngle(const double angle_rad) { heel_contact_angle = angle_rad; }
    bool setToeContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points);
    bool setHeelContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points);
    // void setGaitGeneratorParam(); // TODO: 作るかも

    bool goPos(const Eigen::Isometry3d& target,
               const std::vector<int>& support_link_cycle,
               const std::vector<int>& swing_link_cycle);

    // gopos: 接触のCycleを記述したい
    // void goPos(const rats::coordinates& target, const size_t one_step_count,
    //            const double max_step_length, const double max_rotate_angle,
    //            const std::vector<std::vector<int>>& support_link_cycle,
    //            const std::vector<std::vector<int>>& swing_link_cycle);
};

}

#endif // GAITGENERATORABS_H
