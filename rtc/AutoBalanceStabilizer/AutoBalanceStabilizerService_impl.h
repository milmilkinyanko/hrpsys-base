// -*-C++-*-
#ifndef AUTOBALANCESTABILIZERSERVICESVC_IMPL_H
#define AUTOBALANCESTABILIZERSERVICESVC_IMPL_H

#include "hrpsys/idl/AutoBalanceStabilizerService.hh"

using namespace OpenHRP;

class AutoBalanceStabilizer;

class AutoBalanceStabilizerService_impl 
  : public virtual POA_OpenHRP::AutoBalanceStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  AutoBalanceStabilizerService_impl();
  virtual ~AutoBalanceStabilizerService_impl();
  CORBA::Boolean goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th);
  CORBA::Boolean goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth);
  CORBA::Boolean goStop();
  CORBA::Boolean emergencyStop();
  CORBA::Boolean setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  CORBA::Boolean setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoBalanceStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  void waitFootStepsEarly(CORBA::Double tm);
  CORBA::Boolean startAutoBalancer(const OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs);
  CORBA::Boolean stopAutoBalancer();
  void startStabilizer(void);
  void stopStabilizer(void);
  CORBA::Boolean setGaitGeneratorParam(const OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param);
  CORBA::Boolean getGaitGeneratorParam(OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam_out i_param);
  CORBA::Boolean setAutoBalancerParam(const OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param);
  CORBA::Boolean getAutoBalancerParam(OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam_out i_param);
  void setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param);
  void getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam_out i_param);
  CORBA::Boolean getFootstepParam(OpenHRP::AutoBalanceStabilizerService::FootstepParam_out i_param);
  CORBA::Boolean adjustFootSteps(const OpenHRP::AutoBalanceStabilizerService::Footstep& rfootstep, const OpenHRP::AutoBalanceStabilizerService::Footstep& lfootstep);
  CORBA::Boolean getRemainingFootstepSequence(OpenHRP::AutoBalanceStabilizerService::FootstepSequence_out o_footstep , CORBA::Long& o_current_fs_idx);
  CORBA::Boolean getGoPosFootstepsSequence(CORBA::Double x, CORBA::Double y, CORBA::Double th, OpenHRP::AutoBalanceStabilizerService::FootstepsSequence_out o_footstep);
  CORBA::Boolean releaseEmergencyStop();
  //
  //
  void autobalancestabilizer(AutoBalanceStabilizer *i_autobalancestabilizer);
private:
  AutoBalanceStabilizer *m_autobalancestabilizer;
};				 

#endif
