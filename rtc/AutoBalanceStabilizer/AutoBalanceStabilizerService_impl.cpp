#include "AutoBalanceStabilizerService_impl.h"
#include "AutoBalanceStabilizer.h"

AutoBalanceStabilizerService_impl::AutoBalanceStabilizerService_impl() : m_autobalancestabilizer(NULL)
{
}

AutoBalanceStabilizerService_impl::~AutoBalanceStabilizerService_impl()
{
}

CORBA::Boolean AutoBalanceStabilizerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
  return m_autobalancestabilizer->goPos(x, y, th);
};

// CORBA::Boolean AutoBalanceStabilizerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
// {
//   return m_autobalancestabilizer->goVelocity(vx, vy, vth);
// };

CORBA::Boolean AutoBalanceStabilizerService_impl::goStop()
{
  return m_autobalancestabilizer->goStop();
};

// CORBA::Boolean AutoBalanceStabilizerService_impl::emergencyStop()
// {
//   return m_autobalancestabilizer->emergencyStop();
// };

// CORBA::Boolean AutoBalanceStabilizerService_impl::setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx)
// {
//   return m_autobalancestabilizer->setFootSteps(fss, overwrite_fs_idx);
// }

// CORBA::Boolean AutoBalanceStabilizerService_impl::setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoBalanceStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx)
// {
//   return m_autobalancestabilizer->setFootStepsWithParam(fss, spss, overwrite_fs_idx);
// }

void AutoBalanceStabilizerService_impl::waitFootSteps()
{
  return m_autobalancestabilizer->waitFootSteps();
};

// void AutoBalanceStabilizerService_impl::waitFootStepsEarly(CORBA::Double tm)
// {
//   return m_autobalancestabilizer->waitFootStepsEarly(tm);
// };

CORBA::Boolean AutoBalanceStabilizerService_impl::startAutoBalancer(const OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs)
{
  return m_autobalancestabilizer->startAutoBalancer(limbs);
};

CORBA::Boolean AutoBalanceStabilizerService_impl::stopAutoBalancer()
{
  return m_autobalancestabilizer->stopAutoBalancer();
};

void AutoBalanceStabilizerService_impl::startStabilizer(void)
{
  m_autobalancestabilizer->startStabilizer();
}

void AutoBalanceStabilizerService_impl::stopStabilizer(void)
{
  m_autobalancestabilizer->stopStabilizer();
}

CORBA::Boolean AutoBalanceStabilizerService_impl::setGaitGeneratorParam(const OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param)
{
  return m_autobalancestabilizer->setGaitGeneratorParam(i_param);
};

CORBA::Boolean AutoBalanceStabilizerService_impl::getGaitGeneratorParam(OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam_out i_param)
{
  i_param = new OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam();
  i_param->stride_parameter.length(6);
  i_param->toe_heel_phase_ratio.length(7);
  i_param->zmp_weight_map.length(4);
  return m_autobalancestabilizer->getGaitGeneratorParam(*i_param);
};

CORBA::Boolean AutoBalanceStabilizerService_impl::setAutoBalancerParam(const OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param)
{
  return m_autobalancestabilizer->setAutoBalancerParam(i_param);
};

CORBA::Boolean AutoBalanceStabilizerService_impl::getAutoBalancerParam(OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam_out i_param)
{
  i_param = new OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam();
  return m_autobalancestabilizer->getAutoBalancerParam(*i_param);
};

void AutoBalanceStabilizerService_impl::setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param)
{
  m_autobalancestabilizer->setStabilizerParam(i_param);
}

void AutoBalanceStabilizerService_impl::getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam_out i_param)
{
  i_param = new OpenHRP::AutoBalanceStabilizerService::StabilizerParam();
  return m_autobalancestabilizer->getStabilizerParam(*i_param);
}

CORBA::Boolean AutoBalanceStabilizerService_impl::getFootstepParam(OpenHRP::AutoBalanceStabilizerService::FootstepParam_out i_param)
{
  i_param = new OpenHRP::AutoBalanceStabilizerService::FootstepParam();
  return m_autobalancestabilizer->getFootstepParam(*i_param);
};

// CORBA::Boolean AutoBalanceStabilizerService_impl::adjustFootSteps(const OpenHRP::AutoBalanceStabilizerService::Footstep& rfootstep, const OpenHRP::AutoBalanceStabilizerService::Footstep& lfootstep)
// {
//     return m_autobalancestabilizer->adjustFootSteps(rfootstep, lfootstep);
// };

// CORBA::Boolean AutoBalanceStabilizerService_impl::getRemainingFootstepSequence(OpenHRP::AutoBalanceStabilizerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx)
// {
//     return m_autobalancestabilizer->getRemainingFootstepSequence(o_footstep, o_current_fs_idx);
// };

// CORBA::Boolean AutoBalanceStabilizerService_impl::getGoPosFootstepsSequence(CORBA::Double x, CORBA::Double y, CORBA::Double th, OpenHRP::AutoBalanceStabilizerService::FootstepsSequence_out o_footstep)
// {
//     return m_autobalancestabilizer->getGoPosFootstepsSequence(x, y, th, o_footstep);
// };

CORBA::Boolean AutoBalanceStabilizerService_impl::releaseEmergencyStop()
{
    return m_autobalancestabilizer->releaseEmergencyStop();
};

void AutoBalanceStabilizerService_impl::autobalancestabilizer(AutoBalanceStabilizer *i_autobalancestabilizer)
{
  m_autobalancestabilizer = i_autobalancestabilizer;
}
