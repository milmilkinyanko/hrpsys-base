// -*- C++ -*-
/*!
 * @file  AutoBalanceStabilizer.h
 * @brief autobalancestabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef AUTOBALANCESTABILIZER_H
#define AUTOBALANCESTABILIZER_H

#include <memory>
#include <mutex>
#include <boost/make_shared.hpp>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/CorbaNaming.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <hrpModel/Link.h>
#include <hrpModel/Body.h>
#include <hrpModel/ModelLoaderUtil.h>

#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include "FullbodyInverseKinematicsSolver.h"
#include "GaitGenerator.h"
#include "Stabilizer.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AutoBalanceStabilizerService_impl.h"


// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;
using paramsToStabilizer = paramsFromAutoBalancer;

class AutoBalanceStabilizer : public RTC::DataFlowComponentBase
{
  public:
    AutoBalanceStabilizer(RTC::Manager* manager);
    // virtual ~AutoBalanceStabilizer();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    RTC::ReturnCode_t onInitialize() override;

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    RTC::ReturnCode_t onFinalize() override;

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id) override;

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id) override;

    // The execution action that is invoked periodically
    // former rtc_active_do()
    RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;

    bool goPos(const double& x, const double& y, const double& th);
    bool goVelocity(const double& vx, const double& vy, const double& vth);
    bool goStop();
    bool emergencyStop ();
    bool setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx);
    bool setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
    bool setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepSequence& fs, const OpenHRP::AutoBalanceStabilizerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx);
    bool setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoBalanceStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
    void waitFootSteps();
    void waitFootStepsEarly(const double tm);
    bool startAutoBalancer(const ::OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs);
    bool stopAutoBalancer();
    bool setGaitGeneratorParam(const OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param);
    bool getGaitGeneratorParam(OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param);
    bool setAutoBalancerParam(const OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param);
    bool getAutoBalancerParam(OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param);
    bool getFootstepParam(OpenHRP::AutoBalanceStabilizerService::FootstepParam& i_param);
    bool adjustFootSteps(const OpenHRP::AutoBalanceStabilizerService::Footstep& rfootstep, const OpenHRP::AutoBalanceStabilizerService::Footstep& lfootstep);
    bool getRemainingFootstepSequence(OpenHRP::AutoBalanceStabilizerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx);
    bool getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalanceStabilizerService::FootstepsSequence_out o_footstep);
    bool releaseEmergencyStop();
    void distributeReferenceZMPToWrenches (const hrp::Vector3& _ref_zmp);

    // Stabilizer
    void getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param);
    void setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param);
    void startStabilizer(void);
    void stopStabilizer(void);

  protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">

    // DataInPort declaration
    // <rtc-template block="inport_declare">
    TimedDoubleSeq m_qCurrent;
    InPort<TimedDoubleSeq> m_qCurrentIn;
    TimedDoubleSeq m_qRef;
    InPort<TimedDoubleSeq> m_qRefIn;
    TimedOrientation3D m_rpy;
    InPort<TimedOrientation3D> m_rpyIn;
    TimedPoint3D m_basePos;
    InPort<TimedPoint3D> m_basePosIn;
    TimedOrientation3D m_baseRpy;
    InPort<TimedOrientation3D> m_baseRpyIn;
    TimedPoint3D m_refZmp;
    InPort<TimedPoint3D> m_refZmpIn;
    TimedDoubleSeq m_optionalData;
    InPort<TimedDoubleSeq> m_optionalDataIn;
    std::vector<TimedDoubleSeq> m_ref_force;
    std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
    TimedPoint3D m_refFootOriginExtMoment;
    InPort<TimedPoint3D> m_refFootOriginExtMomentIn;
    TimedBoolean m_refFootOriginExtMomentIsHoldValue;
    InPort<TimedBoolean> m_refFootOriginExtMomentIsHoldValueIn;
    std::vector<TimedDoubleSeq> m_wrenches;
    std::vector<InPort<TimedDoubleSeq> *> m_wrenchesIn;

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    OutPort<TimedDoubleSeq> m_qOut;
    // TimedDoubleSeq m_tau;
    // OutPort<TimedDoubleSeq> m_tauOut;
    OutPort<TimedPoint3D> m_refZmpOut;
    OutPort<TimedPoint3D> m_basePosOut;
    OutPort<TimedOrientation3D> m_baseRpyOut;
    TimedDoubleSeq m_baseTform;
    OutPort<TimedDoubleSeq> m_baseTformOut;
    TimedPose3D m_basePose;
    OutPort<TimedPose3D> m_basePoseOut;
    TimedAcceleration3D m_accRef;
    OutPort<TimedAcceleration3D> m_accRefOut;
    TimedBooleanSeq m_refContactStates;
    OutPort<TimedBooleanSeq> m_refContactStatesOut;
    TimedDoubleSeq m_controlSwingSupportTime;
    OutPort<TimedDoubleSeq> m_controlSwingSupportTimeOut;
    TimedPoint3D m_sbpCogOffset;
    OutPort<TimedPoint3D> m_sbpCogOffsetOut;
    TimedPoint3D m_diffFootOriginExtMoment;
    OutPort<TimedPoint3D> m_diffFootOriginExtMomentOut;
    TimedLong m_emergencySignal;
    OutPort<TimedLong> m_emergencySignalOut;

    // for debug
    TimedPoint3D m_refCog;
    OutPort<TimedPoint3D> m_refCogOut;
    TimedPoint3D m_originNewRefZmp;
    OutPort<TimedPoint3D> m_originNewRefZmpOut;
    TimedPoint3D m_originActZmp;
    OutPort<TimedPoint3D> m_originActZmpOut;
    TimedPoint3D m_footOriginRefCog;
    OutPort<TimedPoint3D> m_footOriginRefCogOut;
    TimedPoint3D m_footOriginActCog;
    OutPort<TimedPoint3D> m_footOriginActCogOut;
    TimedBooleanSeq m_actContactStates;
    OutPort<TimedBooleanSeq> m_actContactStatesOut;
    TimedPoint3D m_refCP;
    OutPort<TimedPoint3D> m_refCPOut;
    TimedPoint3D m_actCP;
    OutPort<TimedPoint3D> m_actCPOut;
    TimedDoubleSeq m_COPInfo;
    OutPort<TimedDoubleSeq> m_COPInfoOut;

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">
    RTC::CorbaPort m_AutoBalanceStabilizerServicePort;

    // Service declaration
    // <rtc-template block="service_declare">
    AutoBalanceStabilizerService_impl m_service0;

  private:
    hrp::BodyPtr m_robot = boost::make_shared<hrp::Body>();
    double m_dt;
    std::mutex m_mutex;
    unsigned int m_debugLevel = 0;
    unsigned int loop = 0;

    // Fullbody Inverse Kinematics Solver
    size_t max_ik_iteration = 10;
    std::vector<hrp::IKConstraint> ik_constraints;
    std::unique_ptr<hrp::FullbodyInverseKinematicsSolver> fik;

    inline bool loadModel(hrp::BodyPtr body, const string& model_path);

    // -- Functions for OpenRTM port --
    inline void setupBasicPort();
    inline void readInportData();
    inline void writeOutPortData(const hrp::Vector3& base_pos,
                                 const hrp::Matrix33& base_rot,
                                 const hrp::Vector3& ref_zmp_base_frame,
                                 const hrp::Vector3& ref_cog,
                                 const hrp::Vector3& sbp_cog_offset,
                                 const hrp::Vector3& acc_ref,
                                 const stabilizerLogData& st_log_data);

    // -- Functions for OpenRTM port --
    std::vector<hrp::LinkConstraint> readContactPointsFromProps(const RTC::Properties& prop);
    void addBodyConstraint(std::vector<hrp::LinkConstraint>& constraints,
                           const hrp::BodyPtr& _robot);
    void setupIKConstraints(const hrp::BodyPtr& _robot,
                            const std::vector<hrp::LinkConstraint>& constraints);
    void setIKConstraintsTarget();



    void writeOutportDataForLeggedRobot();
    void getTargetParameters();
    void getActualParameters();
    void solveFullbodyIK ();
    void startABCparam(const ::OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs);
    void stopABCparam();
    void waitABCTransition();
    // Functions to calculate parameters for ABC output.
    // Output parameters are EE, limbCOPOffset, contactStates, controlSwingSupportTime, toeheelPhaseRation
    void interpolateLegNamesAndZMPOffsets();
    // void updateTargetCoordsForHandFixMode (rats::coordinates& tmp_fix_coords);
    void calculateOutputRefForces ();
    hrp::Matrix33 OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2);
    void fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot);
    void fixLegToCoords2 (rats::coordinates& tmp_fix_coords);
    bool startWalking ();
    void stopWalking ();
    // static balance point offsetting
    void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
    void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height);
    hrp::Vector3 calc_vel_from_hand_error (const rats::coordinates& tmp_fix_coords);
    // bool isOptionalDataContact (const std::string& ee_name)
    // {
    //     return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]] - 1.0) < 0.1) ? true : false;
    // }
    std::string getUseForceModeString ();

    // member variables to store values for data ports
    hrp::dvector q_current; // TODO: m_robot_act があればいらない
    hrp::dvector q_ref; // TODO: m_robotだけで十分?
    hrp::Vector3 act_rpy; // TODO: m_robot_act があればいらない ?
    hrp::Vector3 ref_base_pos; // TODO: m_robotだけで十分?
    hrp::Matrix33 ref_base_rot; // TODO: m_robotだけで十分?

    hrp::Vector3 input_ref_zmp;
    std::vector<hrp::dvector6> ref_wrenches;
    std::vector<hrp::dvector6> ref_wrenches_for_st;
    std::vector<hrp::dvector6> act_wrenches;

    std::vector<bool> ref_contact_states; // TODO: delete
    std::vector<double> control_swing_support_time; // TODO: delete
    hrp::Vector3 ref_foot_origin_ext_moment;
    bool is_ref_foot_origin_ext_moment_hold_value;

    std::unique_ptr<Stabilizer> st;

    // for gg
    std::unique_ptr<hrp::GaitGenerator> gg;
    bool gg_is_walking, gg_solved;

    // for abc
    hrp::Vector3 ref_cog, ref_zmp, prev_ref_zmp, prev_imu_sensor_pos, prev_imu_sensor_vel, hand_fix_initial_offset;
    // enum {BIPED, TROT, PACE, CRAWL, GALLOP} gait_type;
    enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::vector<std::string> sensor_names, leg_names, ee_vec;
    Eigen::Isometry3d target_root;
    rats::coordinates fix_leg_coords, fix_leg_coords2; // TODO: rename
    double d_pos_z_root, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2];
    bool use_limb_stretch_avoidance;

    double transition_interpolator_ratio, transition_time, zmp_transition_time, adjust_footstep_transition_time, leg_names_interpolator_ratio;
    std::unique_ptr<interpolator> zmp_offset_interpolator;
    std::unique_ptr<interpolator> transition_interpolator;
    std::unique_ptr<interpolator> adjust_footstep_interpolator;
    std::unique_ptr<interpolator> leg_names_interpolator;

    // static balance point offsetting
    hrp::Vector3 sbp_offset, sbp_cog_offset;
    enum {MODE_NO_FORCE, MODE_REF_FORCE, MODE_REF_FORCE_WITH_FOOT, MODE_REF_FORCE_RFU_EXT_MOMENT} use_force;
    std::vector<hrp::Vector3> ref_forces, ref_moments;

    bool is_legged_robot, is_stop_mode, is_hand_fix_mode, is_hand_fix_initial;
    bool graspless_manip_mode;
    std::string graspless_manip_arm;
    hrp::Vector3 graspless_manip_p_gain;
    rats::coordinates graspless_manip_reference_trans_coords;

    hrp::InvDynStateBuffer idsb;
    std::vector<IIRFilter> invdyn_zmp_filters;

    // Used for ref force balancing.
    hrp::Link* additional_force_applied_link; // TODO: raw pointer?
    hrp::Vector3 additional_force_applied_point_offset;
};


extern "C"
{
    void AutoBalanceStabilizerInit(RTC::Manager* manager);
};

#endif // AUTOBALANCESTABILIZER_H
