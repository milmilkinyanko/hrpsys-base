// -*- mode: C++ -*-

/**
 * @file  StateEstimator.h
 * @brief
 * @date  $Date$
 */

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <cmath>
#include <mutex>
#include <Eigen/Geometry>
#include <hrpUtil/EigenTypes.h>
#include <hrpModel/Body.h>
#include "LinkConstraint.h"
#include "../TorqueFilter/IIRFilter.h"

namespace hrp {

struct stateActInputData
{
    const hrp::dvector& q_current;
    const hrp::Vector3& rpy;
    const hrp::ConstraintsWithCount& constraints;
    double zmp_z; // 常にreferenceの値を用いる
    size_t cur_const_idx;
};

struct stateRefInputData
{
    const std::vector<ConstraintsWithCount>& constraints_list;
    size_t cur_const_idx;
    const hrp::Vector3& base_frame_zmp;

    // 状態推定には使われず値を保持してるだけ
    bool is_walking;
    const hrp::Vector3& sbp_cog_offset;
};

struct limbParam
{
    hrp::Vector3 contact_cop_info = hrp::Vector3::Zero();
    double prev_act_force_z = 0;
    bool contact_states = false;
    Eigen::Isometry3d foot_frame_ee_coord = Eigen::Isometry3d::Identity();
    hrp::dvector6 wrenches = hrp::dvector6::Zero();

    // only for reference
    double control_swing_support_time = 1.0;
};

class StateEstimator
{
  private:
    hrp::BodyPtr m_robot;
    std::mutex& m_mutex; // This is the reference to the mutex of AutoBalanceStabilizer class
    const std::string comp_name;
    const double dt;
    static constexpr double g_acc = 9.80665; // [m/s^2]

    // world frame
    hrp::Vector3 cog = hrp::Vector3::Zero();
    hrp::Vector3 zmp = hrp::Vector3::Zero();
    hrp::Vector3 base_rpy = hrp::Vector3::Zero();
    hrp::Vector3 imu_rpy = hrp::Vector3::Zero();

    // base-link frame
    hrp::Vector3 base_frame_zmp = hrp::Vector3::Zero();
    hrp::Vector3 base_frame_cp = hrp::Vector3::Zero();

    // foot-origion frame
    hrp::Vector3 foot_frame_cog = hrp::Vector3::Zero();
    hrp::Vector3 prev_foot_frame_cog = hrp::Vector3::Zero();
    hrp::Vector3 foot_frame_zmp = hrp::Vector3::Zero();
    hrp::Vector3 foot_frame_cogvel = hrp::Vector3::Zero();
    hrp::Vector3 foot_frame_cp = hrp::Vector3::Zero();

    Eigen::Isometry3d foot_origin_coord = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d prev_foot_origin_coord = Eigen::Isometry3d::Identity();

    std::map<int, limbParam> limb_param;
    bool on_ground = false;
    bool prev_on_ground = false;
    double contact_decision_threshold = 25; // [N]

    size_t prev_const_idx = 0;
    size_t jump_time_count = 0;
    double jump_initial_velocity_z = 0;
    std::unique_ptr<FirstOrderLowPassFilter<hrp::Vector3>> cogvel_filter;

  public:
    StateEstimator(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt, std::mutex& _mutex, const std::vector<int>& link_indices);

    // for actual
    void calcActStates(const stateActInputData& input_data);
    bool calcZMP(hrp::Vector3& ret_zmp, const hrp::ConstraintsWithCount& constraints, const double zmp_z);
    inline bool isContact(const int idx) { return limb_param[idx].prev_act_force_z > contact_decision_threshold; };

    // for reference
    void calcRefStates(const stateRefInputData& input_data, const size_t cur_count);
    double calcSwingSupportTime(const std::vector<ConstraintsWithCount>& constraints_list, const size_t cur_const_idx, const size_t limb_idx, const size_t cur_count);

    // getter
    const hrp::Vector3& getZmp() { return zmp; };
    const hrp::Vector3& getIMURpy() { return imu_rpy; };
    const hrp::Vector3& getBaseRpy() { return base_rpy; };
    const hrp::Vector3& getBaseFrameZmp() { return base_frame_zmp; };
    const hrp::Vector3& getBaseFrameCp() { return base_frame_cp; };
    const hrp::Vector3& getFootFrameCog() { return foot_frame_cog; };
    const hrp::Vector3& getFootFrameZmp() { return foot_frame_zmp; };
    const hrp::Vector3& getFootFrameCogVel() { return foot_frame_cogvel; };
    const hrp::Vector3& getFootFrameCp() { return foot_frame_cp; };

    bool getOnGround() { return on_ground; };

    const Eigen::Isometry3d& getFootOriginCoord() { return foot_origin_coord; };
    const Eigen::Isometry3d::TranslationPart getFootOriginPos() { return foot_origin_coord.translation(); };
    const Eigen::Isometry3d::LinearPart getFootOriginRot() { return foot_origin_coord.linear(); };

    double getCogVelCutOffFreq() { return cogvel_filter->getCutOffFreq(); };

    // getter for each limb
    const hrp::dvector6& getWrenches(const int idx) { return limb_param[idx].wrenches; };
    bool getContactStates(const int idx) { return limb_param[idx].contact_states; };
    double getControlSwingSupportTime(const int idx) { return limb_param[idx].control_swing_support_time; };

    const Eigen::Isometry3d& getFootFrameEECoord(const int idx) { return limb_param[idx].foot_frame_ee_coord; };
    const Eigen::Isometry3d::TranslationPart getFootFrameEEPos(const int idx) { return limb_param[idx].foot_frame_ee_coord.translation(); };
    const Eigen::Isometry3d::LinearPart getFootFrameEERot(const int idx) { return limb_param[idx].foot_frame_ee_coord.linear(); };

    // setter
    void setCogVelCutOffFreq(const double freq) { cogvel_filter->setCutOffFreq(freq); };
};

}

#endif // STATEESTIMATOR_H
