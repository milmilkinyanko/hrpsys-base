// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "Stabilizer.h"
#include "hrpsys/util/VectorConvert.h"
#include <math.h>
#include <boost/lambda/lambda.hpp>

typedef coil::Guard<coil::Mutex> Guard;

#ifndef deg2rad
#define deg2rad(x) ((x) * M_PI / 180.0)
#endif
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif

void Stabilizer::waitSTTransition()
{
  // Wait condition
  //   1. Check transition_count : Wait until transition is finished
  //   2. Check control_mode : Once control_mode is SYNC mode, wait until control_mode moves to the next mode (MODE_AIR or MODE_IDLE)
  bool flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
  while (transition_count != 0 ||
         (flag ? !(control_mode == MODE_IDLE || control_mode == MODE_AIR) : false) ) {
      usleep(10);
      flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
  }
  usleep(10);
}

void Stabilizer::sync_2_st ()
{
  std::cerr << "[" << print_str << "] ["
            << "] Sync IDLE => ST"  << std::endl;
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  d_rpy[0] = d_rpy[1] = 0;
  pdr = hrp::Vector3::Zero();
  pos_ctrl = hrp::Vector3::Zero();
  for (size_t i = 0; i < stikp.size(); i++) {
    STIKParam& ikp = stikp[i];
    ikp.target_ee_diff_p = hrp::Vector3::Zero();
    ikp.target_ee_diff_r = hrp::Matrix33::Identity();
    ikp.d_pos_swing = ikp.prev_d_pos_swing = hrp::Vector3::Zero();
    ikp.d_rpy_swing = ikp.prev_d_rpy_swing = hrp::Vector3::Zero();
    ikp.target_ee_diff_p_filter->reset(hrp::Vector3::Zero());
    ikp.target_ee_diff_r_filter->reset(hrp::Vector3::Zero());
    ikp.d_foot_pos = ikp.d_foot_rpy = ikp.ee_d_foot_rpy = hrp::Vector3::Zero();
  }
  if (on_ground) {
    transition_count = -1 * calcMaxTransitionCount();
    control_mode = MODE_ST;
  } else {
    transition_count = 0;
    control_mode = MODE_AIR;
  }
}

void Stabilizer::sync_2_idle ()
{
  std::cerr << "[" << print_str << "] ["
            << "] Sync ST => IDLE"  << std::endl;
  transition_count = calcMaxTransitionCount();
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
}

void Stabilizer::startStabilizer(void)
{
    waitSTTransition(); // Wait until all transition has finished
    {
        Guard guard(m_mutex);
        if ( control_mode == MODE_IDLE ) {
            std::cerr << "[" << print_str << "] " << "Start ST"  << std::endl;
            sync_2_st();
        }
    }
    waitSTTransition();
    std::cerr << "[" << print_str << "] " << "Start ST DONE"  << std::endl;
}

void Stabilizer::stopStabilizer(void)
{
    waitSTTransition(); // Wait until all transition has finished
    {
        Guard guard(m_mutex);
        if ( (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
            std::cerr << "[" << print_str << "] " << "Stop ST"  << std::endl;
            control_mode = (control_mode == MODE_ST) ? MODE_SYNC_TO_IDLE : MODE_IDLE;
        }
    }
    waitSTTransition();
    std::cerr << "[" << print_str << "] " << "Stop ST DONE"  << std::endl;
}
