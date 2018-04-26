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

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DEBUGP2 (loop%10==0)

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

// Damping control functions
//   Basically Equation (14) in the paper [1]
double Stabilizer::calcDampingControl (const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
  return (1/DD * (tau_d - tau) - 1/TT * prev_d) * dt + prev_d;
};

// Retrieving only
hrp::Vector3 Stabilizer::calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT)
{
  return (- prev_d.cwiseQuotient(TT)) * dt + prev_d;
};

// Retrieving only
double Stabilizer::calcDampingControl (const double prev_d, const double TT)
{
  return - 1/TT * prev_d * dt + prev_d;
};

hrp::Vector3 Stabilizer::calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                             const hrp::Vector3& DD, const hrp::Vector3& TT)
{
  return ((tau_d - tau).cwiseQuotient(DD) - prev_d.cwiseQuotient(TT)) * dt + prev_d;
};

void Stabilizer::calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p)
{
  // tm.resize(6,6*contact_p.size());
  // tm.setZero();
  // for (size_t c = 0; c < contact_p.size(); c++) {
  //   for (size_t i = 0; i < 6; i++) tm(i,(c*6)+i) = 1.0;
  //   hrp::Matrix33 cm;
  //   rats::outer_product_matrix(cm, contact_p[c]);
  //   for (size_t i = 0; i < 3; i++)
  //     for (size_t j = 0; j < 3; j++) tm(i+3,(c*6)+j) = cm(i,j);
  // }
}

void Stabilizer::calcTorque ()
{
  m_robot->calcForwardKinematics();
  // buffers for the unit vector method
  hrp::Vector3 root_w_x_v;
  hrp::Vector3 g(0, 0, 9.80665);
  root_w_x_v = m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));
  m_robot->rootLink()->dvo = g - root_w_x_v;   // dv = g, dw = 0
  m_robot->rootLink()->dw.setZero();

  hrp::Vector3 root_f;
  hrp::Vector3 root_t;
  m_robot->calcInverseDynamics(m_robot->rootLink(), root_f, root_t);
  // if (loop % 200 == 0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   std::cerr << ":cog "; rats::print_vector(std::cerr, m_robot->calcCM());
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->link(i)->name << " "
  //               << m_robot->joint(i)->jointId << " "
  //               << m_robot->link(i)->m << " ";
  //     hrp::Vector3 tmpc = m_robot->link(i)->p + m_robot->link(i)->R * m_robot->link(i)->c;
  //     rats::print_vector(std::cerr, tmpc, false);
  //     std::cerr << " ";
  //     rats::print_vector(std::cerr, m_robot->link(i)->c, false);
  //     std::cerr << ")" << std::endl;
  //   }
  // }
  // if (loop % 200 == 0) {
  //   std::cerr << ":IV1 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
  hrp::dmatrix contact_mat, contact_mat_inv;
  std::vector<hrp::Vector3> contact_p;
  for (size_t j = 0; j < 2; j++) contact_p.push_back(m_robot->sensor<hrp::ForceSensor>(stikp[j].sensor_name)->link->p);
  calcContactMatrix(contact_mat, contact_p);
  hrp::calcSRInverse(contact_mat, contact_mat_inv, 0.0);
  hrp::dvector root_ft(6);
  for (size_t j = 0; j < 3; j++) root_ft(j) = root_f(j);
  for (size_t j = 0; j < 3; j++) root_ft(j+3) = root_t(j);
  hrp::dvector contact_ft(2*6);
  contact_ft = contact_mat_inv * root_ft;
  // if (loop%200==0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   // std::cerr << ":ftv "; rats::print_vector(std::cerr, ftv);
  //   // std::cerr << ":aa "; rats::print_matrix(std::cerr, aa);
  //   // std::cerr << ":dv "; rats::print_vector(std::cerr, dv);
  // }
  for (size_t j = 0; j < 2; j++) {
    hrp::JointPathEx jm = hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(stikp[j].sensor_name)->link, dt);
    hrp::dmatrix JJ;
    jm.calcJacobian(JJ);
    hrp::dvector ft(6);
    for (size_t i = 0; i < 6; i++) ft(i) = contact_ft(i+j*6);
    hrp::dvector tq_from_extft(jm.numJoints());
    tq_from_extft = JJ.transpose() * ft;
    // if (loop%200==0) {
    //   std::cerr << ":ft "; rats::print_vector(std::cerr, ft);
    //   std::cerr << ":JJ "; rats::print_matrix(std::cerr, JJ);
    //   std::cerr << ":tq_from_extft "; rats::print_vector(std::cerr, tq_from_extft);
    // }
    for (size_t i = 0; i < jm.numJoints(); i++) jm.joint(i)->u -= tq_from_extft(i);
  }
  //hrp::dmatrix MM(6,m_robot->numJoints());
  //m_robot->calcMassMatrix(MM);
  // if (loop % 200 == 0) {
  //   std::cerr << ":INVDYN2 (list "; rats::print_vector(std::cerr, root_f, false);
  //   std::cerr << " "; rats::print_vector(std::cerr, root_t, false);
  //   std::cerr << ")" << std::endl;
  //   // hrp::dvector tqv(m_robot->numJoints());
  //   // for(int i = 0; i < m_robot->numJoints(); ++i){p
  //   //   tqv[m_robot->joint(i)->jointId] = m_robot->joint(i)->u;
  //   // }
  //   // std::cerr << ":IV2 "; rats::print_vector(std::cerr, tqv);
  //   std::cerr << ":IV2 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
};

void Stabilizer::calcRUNST() {
  if ( m_robot->numJoints() == qRef.size() ) {
    std::vector<std::string> target_name;
    target_name.push_back("L_ANKLE_R");
    target_name.push_back("R_ANKLE_R");

    double angvelx_ref;// = (m_rpyRef.data.r - pangx_ref)/dt;
    double angvely_ref;// = (m_rpyRef.data.p - pangy_ref)/dt;
    //pangx_ref = m_rpyRef.data.r;
    //pangy_ref = m_rpyRef.data.p;
    double angvelx = (rpy(0) - pangx)/dt;
    double angvely = (rpy(1) - pangy)/dt;
    pangx = rpy(0);
    pangy = rpy(1);

    // update internal robot model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      qorg[i] = m_robot->joint(i)->q;
      m_robot->joint(i)->q = qRef[i];
      qrefv[i] = qRef[i];
    }
    //double orgjq = m_robot->link("L_FOOT")->joint->q;
    double orgjq = m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q;
    //set root
    m_robot->rootLink()->p = hrp::Vector3(0,0,0);
    //m_robot->rootLink()->R = hrp::rotFromRpy(m_rpyRef.data.r,m_rpyRef.data.p,m_rpyRef.data.y);
    m_robot->calcForwardKinematics();
    hrp::Vector3 target_root_p = m_robot->rootLink()->p;
    hrp::Matrix33 target_root_R = m_robot->rootLink()->R;
    hrp::Vector3 target_foot_p[2];
    hrp::Matrix33 target_foot_R[2];
    for (size_t i = 0; i < 2; i++) {
      target_foot_p[i] = m_robot->link(target_name[i])->p;
      target_foot_R[i] = m_robot->link(target_name[i])->R;
    }
    hrp::Vector3 target_fm = (m_robot->link(target_name[0])->p + m_robot->link(target_name[1])->p)/2;
    //hrp::Vector3 org_cm = m_robot->rootLink()->R.transpose() * (m_robot->calcCM() - m_robot->rootLink()->p);
    hrp::Vector3 org_cm = m_robot->rootLink()->R.transpose() * (target_fm - m_robot->rootLink()->p);

    // stabilizer loop
    if ( ( wrenches[1].size() > 0 && wrenches[0].size() > 0 )
         //( m_wrenches[ST_LEFT].data[2] > m_robot->totalMass()/4 || m_wrenches[ST_RIGHT].data[2] > m_robot->totalMass()/4 )
         ) {

      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = qorg[i];
      }
      // set root
      double rddx;// = k_run_b[0] * (m_rpyRef.data.r - m_rpy.data.r) + d_run_b[0] * (angvelx_ref - angvelx);
      double rddy;// = k_run_b[1] * (m_rpyRef.data.p - m_rpy.data.p) + d_run_b[1] * (angvely_ref - angvely);
      rdx += rddx * dt;
      rx += rdx * dt;
      rdy += rddy * dt;
      ry += rdy * dt;
      //rx += rddx * dt;
      //ry += rddy * dt;
      // if (DEBUGP2) {
      //   std::cerr << "REFRPY " <<  m_rpyRef.data.r << " " << m_rpyRef.data.p << std::endl;
      // }
      // if (DEBUGP2) {
      //   std::cerr << "RPY " <<  m_rpy.data.r << " " << m_rpy.data.p << std::endl;
      //   std::cerr << " rx " << rx << " " << rdx << " " << rddx << std::endl;
      //   std::cerr << " ry " << ry << " " << rdy << " " << rddy << std::endl;
      // }
      hrp::Vector3 root_p_s;
      hrp::Matrix33 root_R_s;
      rats::rotm3times(root_R_s, hrp::rotFromRpy(rx, ry, 0), target_root_R);
      if (DEBUGP2) {
        hrp::Vector3 tmp = hrp::rpyFromRot(root_R_s);
        std::cerr << "RPY2 " <<  tmp(0) << " " << tmp(1) << std::endl;
      }
      root_p_s = target_root_p + target_root_R * org_cm - root_R_s * org_cm;
      //m_robot->calcForwardKinematics();
      // FK
      m_robot->rootLink()->R = root_R_s;
      m_robot->rootLink()->p = root_p_s;
      if (DEBUGP2) {
        std::cerr << " rp " << root_p_s[0] << " " << root_p_s[1] << " " << root_p_s[2] << std::endl;
      }
      m_robot->calcForwardKinematics();
      //
      hrp::Vector3 current_fm = (m_robot->link(target_name[0])->p + m_robot->link(target_name[1])->p)/2;

      // 3D-LIP model contorller
      hrp::Vector3 dr = target_fm - current_fm;
      //hrp::Vector3 dr = current_fm - target_fm ;
      hrp::Vector3 dr_vel = (dr - pdr)/dt;
      pdr = dr;
      double tau_y = - m_torque_k[0] * dr(0) - m_torque_d[0] * dr_vel(0);
      double tau_x = m_torque_k[1] * dr(1) + m_torque_d[1] * dr_vel(1);
      if (DEBUGP2) {
        dr*=1e3;
        dr_vel*=1e3;
        std::cerr << "dr " << dr(0) << " " << dr(1) << " " << dr_vel(0) << " " << dr_vel(1) << std::endl;
        std::cerr << "tau_x " << tau_x << std::endl;
        std::cerr << "tau_y " << tau_y << std::endl;
      }

      double gamma = 0.5; // temp
      double tau_xl[2];
      double tau_yl[2];
      double xfront = 0.125;
      double xrear = 0.1;
      double yin = 0.02;
      double yout = 0.15;
      double mg = m_robot->totalMass() * 9.8 * 0.9;// margin
      double tq_y_ulimit = mg * xrear;
      double tq_y_llimit = -1 * mg * xfront;
      double tq_x_ulimit = mg * yout;
      double tq_x_llimit = mg * yin;
      // left
      tau_xl[0] = gamma * tau_x;
      tau_yl[0] = gamma * tau_y;
      tau_xl[0] = vlimit(tau_xl[0], tq_x_llimit, tq_x_ulimit);
      tau_yl[0] = vlimit(tau_yl[0], tq_y_llimit, tq_y_ulimit);
      // right
      tau_xl[1]= (1- gamma) * tau_x;
      tau_yl[1]= (1- gamma) * tau_y;
      tau_xl[1] = vlimit(tau_xl[1], -1*tq_x_ulimit, -1*tq_x_llimit);
      tau_yl[1] = vlimit(tau_yl[1], tq_y_llimit, tq_y_ulimit);

      double dleg_x[2];
      double dleg_y[2];
      double tau_y_total = (wrenches[1][4] + wrenches[0][4]) / 2;
      double dpz;
      if (DEBUGP2) {
        std::cerr << "tq limit " << tq_x_ulimit << " " << tq_x_llimit << " " << tq_y_ulimit << " " << tq_y_llimit << std::endl;
      }
      for (size_t i = 0; i < 2; i++) {
        // dleg_x[i] = m_tau_x[i].update(m_wrenches[i].data[3], tau_xl[i]);
        // dleg_y[i] = m_tau_y[i].update(m_wrenches[i].data[4], tau_yl[i]);
        //dleg_x[i] = m_tau_x[i].update(m_wrenches[i].data[3], tau_xl[i]);
        dleg_x[i] = m_tau_x[i].update(0,0);
        dleg_y[i] = m_tau_y[i].update(tau_y_total, tau_yl[i]);
        if (DEBUGP2) {
          std::cerr << i << " dleg_x " << dleg_x[i] << std::endl;
          std::cerr << i << " dleg_y " << dleg_y[i] << std::endl;
          std::cerr << i << " t_x " << wrenches[i][3] << " "<< tau_xl[i] << std::endl;
          std::cerr << i << " t_y " << wrenches[i][4] << " "<< tau_yl[i] << std::endl;
        }
      }

      // calc leg rot
      hrp::Matrix33 target_R[2];
      hrp::Vector3 target_p[2];
      for (size_t i = 0; i < 2; i++) {
        //rats::rotm3times(target_R[i], hrp::rotFromRpy(dleg_x[i], dleg_y[i], 0), target_foot_R[i]);
        rats::rotm3times(target_R[i], hrp::rotFromRpy(0, dleg_y[i], 0), target_foot_R[i]);
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        target_p[i] = target_foot_p[i];
      }
      // 1=>left, 2=>right
      double refdfz = 0;
      dpz = m_f_z.update((wrenches[0][2] - wrenches[1][2]), refdfz);
      //target_p[0](2) = target_foot_p[0](2) + dpz/2;
      //target_p[1](2) = target_foot_p[1](2) - dpz/2;
      target_p[0](2) = target_foot_p[0](2);
      target_p[1](2) = target_foot_p[1](2);

      // IK
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->link(target_name[i]);
        hrp::Vector3 vel_p, vel_r;
        vel_p = target_p[i] - target->p;
        rats::difference_rotation(vel_r, target->R, target_R[i]);
        //jpe_v[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, DEBUGP);
        //jpe_v[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, false);
        //m_robot->joint(m_robot->link(target_name[i])->jointId)->q = dleg_y[i] + orgjq;
      }
      // m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq + m_rpy.data.p;
      // m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq + m_rpy.data.p;
      m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq;
      m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq;
    } else {
      // reinitialize
      for (int i = 0; i < 2; i++) {
        m_tau_x[i].reset();
        m_tau_y[i].reset();
        m_f_z.reset();
      }
    }
  }
}

void Stabilizer::moveBasePosRotForBodyRPYControl ()
{
  // Body rpy control
  //   Basically Equation (1) and (2) in the paper [1]
  hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
  bool is_root_rot_limit = false;
  for (size_t i = 0; i < 2; i++) {
    d_rpy[i] = transition_smooth_gain * (eefm_body_attitude_control_gain[i] * (ref_root_rpy(i) - act_base_rpy(i)) - 1/eefm_body_attitude_control_time_const[i] * d_rpy[i]) * dt + d_rpy[i];
    d_rpy[i] = vlimit(d_rpy[i], -1 * root_rot_compensation_limit[i], root_rot_compensation_limit[i]);
    is_root_rot_limit = is_root_rot_limit || (std::fabs(std::fabs(d_rpy[i]) - root_rot_compensation_limit[i] ) < 1e-5); // near the limit
  }
  rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
  m_robot->rootLink()->R = current_root_R;
  m_robot->rootLink()->p = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
  m_robot->calcForwardKinematics();
  current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
  current_base_pos = m_robot->rootLink()->p;
  if ( DEBUGP || (is_root_rot_limit && loop%200==0) ) {
    std::cerr << "[" << print_str << "] Root rot control" << std::endl;
    if (is_root_rot_limit) std::cerr << "[" << print_str << "]   Root rot limit reached!!" << std::endl;
    std::cerr << "[" << print_str << "]   ref = [" << rad2deg(ref_root_rpy(0)) << " " << rad2deg(ref_root_rpy(1)) << "], "
              << "act = [" << rad2deg(act_base_rpy(0)) << " " << rad2deg(act_base_rpy(1)) << "], "
              << "cur = [" << rad2deg(current_base_rpy(0)) << " " << rad2deg(current_base_rpy(1)) << "], "
              << "limit = [" << rad2deg(root_rot_compensation_limit[0]) << " " << rad2deg(root_rot_compensation_limit[1]) << "][deg]" << std::endl;
  }
};

double Stabilizer::vlimit(double value, double llimit_value, double ulimit_value)
{
  if (value > ulimit_value) {
    return ulimit_value;
  } else if (value < llimit_value) {
    return llimit_value;
  }
  return value;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, double llimit_value, double ulimit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
    if (value(i) > ulimit_value) {
      ret(i) = ulimit_value;
    } else if (value(i) < llimit_value) {
      ret(i) = llimit_value;
    } else {
      ret(i) = value(i);
    }
  }
  return ret;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, const hrp::Vector3& limit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
    if (value(i) > limit_value(i)) {
      ret(i) = limit_value(i);
    } else if (value(i) < -1 * limit_value(i)) {
      ret(i) = -1 * limit_value(i);
    } else {
      ret(i) = value(i);
    }
  }
  return ret;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, const hrp::Vector3& llimit_value, const hrp::Vector3& ulimit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
    if (value(i) > ulimit_value(i)) {
      ret(i) = ulimit_value(i);
    } else if (value(i) < llimit_value(i)) {
      ret(i) = llimit_value(i);
    } else {
      ret(i) = value(i);
    }
  }
  return ret;
}

void Stabilizer::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  for (size_t i = 0; i < stikp.size(); i++) {
    if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
    hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name)->link;
    leg_c[i].pos = target->p + target->R * foot_origin_offset[i];
    hrp::Vector3 xv1(target->R * ex);
    xv1(2)=0.0;
    xv1.normalize();
    hrp::Vector3 yv1(ez.cross(xv1));
    leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
    leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
    leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);
  }
  if (ref_contact_states[contact_states_index_map["rleg"]] &&
      ref_contact_states[contact_states_index_map["lleg"]]) {
    rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
    foot_origin_pos = tmpc.pos;
    foot_origin_rot = tmpc.rot;
  } else if (ref_contact_states[contact_states_index_map["rleg"]]) {
    foot_origin_pos = leg_c[contact_states_index_map["rleg"]].pos;
    foot_origin_rot = leg_c[contact_states_index_map["rleg"]].rot;
  } else {
    foot_origin_pos = leg_c[contact_states_index_map["lleg"]].pos;
    foot_origin_rot = leg_c[contact_states_index_map["lleg"]].rot;
  }
}

bool Stabilizer::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
  double tmpzmpx = 0;
  double tmpzmpy = 0;
  double tmpfz = 0, tmpfz2 = 0.0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (!is_zmp_calc_enable[i]) continue;
    hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
    hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
    hrp::Matrix33 tmpR;
    rats::rotm3times(tmpR, sensor->link->R, sensor->localR);
    hrp::Vector3 nf = tmpR * hrp::Vector3(wrenches[i][0], wrenches[i][1], wrenches[i][2]);
    hrp::Vector3 nm = tmpR * hrp::Vector3(wrenches[i][3], wrenches[i][4], wrenches[i][5]);
    tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
    tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
    tmpfz += nf(2);
    // calc ee-local COP
    hrp::Link* target = m_robot->link(stikp[i].target_name);
    hrp::Matrix33 eeR = target->R * stikp[i].localR;
    hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
    nf = eeR.transpose() * nf;
    nm = eeR.transpose() * nm;
    // ee-local total moment and total force at ee position
    double tmpcopmy = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
    double tmpcopmx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
    double tmpcopfz = nf(2);
    COPInfo[i*3] = tmpcopmx;
    COPInfo[i*3+1] = tmpcopmy;
    COPInfo[i*3+2] = tmpcopfz;
    prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
    tmpfz2 += prev_act_force_z[i];
  }
  if (tmpfz2 < contact_decision_threshold) {
    ret_zmp = act_zmp;
    return false; // in the air
  } else {
    ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    return true; // on ground
  }
};

void Stabilizer::calcStateForEmergencySignal()
{
  // COP Check
  bool is_cop_outside = false;
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] Check Emergency State (seq = " << (is_seq_interpolating?"interpolating":"empty") << ")" << std::endl;
  }
  if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
    if (DEBUGP) {
      std::cerr << "[" << print_str << "] COP check" << std::endl;
    }
    for (size_t i = 0; i < stikp.size(); i++) {
      if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
      // check COP inside
      if (COPInfo[i*3+2] > 20.0 ) {
        hrp::Vector3 tmpcop(COPInfo[i*3+1]/COPInfo[i*3+2], COPInfo[i*3]/COPInfo[i*3+2], 0);
        is_cop_outside = is_cop_outside ||
          (!szd->is_inside_foot(tmpcop, stikp[i].ee_name=="lleg", cop_check_margin) ||
           szd->is_front_of_foot(tmpcop, cop_check_margin) ||
           szd->is_rear_of_foot(tmpcop, cop_check_margin));
        if (DEBUGP) {
          std::cerr << "[" << print_str << "]   [" << stikp[i].ee_name << "] "
                    << "outside(" << !szd->is_inside_foot(tmpcop, stikp[i].ee_name=="lleg", cop_check_margin) << ") "
                    << "front(" << szd->is_front_of_foot(tmpcop, cop_check_margin) << ") "
                    << "rear(" << szd->is_rear_of_foot(tmpcop, cop_check_margin) << ")" << std::endl;
        }
      } else {
        is_cop_outside = true;
      }
    }
  } else {
    is_cop_outside = false;
  }
  // CP Check
  bool is_cp_outside = false;
  if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
    Eigen::Vector2d tmp_cp = act_cp.head(2);
    szd->get_margined_vertices(margined_support_polygon_vetices);
    szd->calc_convex_hull(margined_support_polygon_vetices, act_contact_states, rel_ee_pos, rel_ee_rot);
    if (!is_walking || is_estop_while_walking) is_cp_outside = !szd->is_inside_support_polygon(tmp_cp, - sbp_cog_offset);
    if (DEBUGP) {
      std::cerr << "[" << print_str << "] CP value " << "[" << act_cp(0) << "," << act_cp(1) << "] [m], "
                << "sbp cog offset [" << sbp_cog_offset(0) << " " << sbp_cog_offset(1) << "], outside ? "
                << (is_cp_outside?"Outside":"Inside")
                << std::endl;
    }
    if (is_cp_outside) {
      if (initial_cp_too_large_error || loop % static_cast <int>(0.2/dt) == 0 ) { // once per 0.2[s]
        std::cerr << "[" << print_str << "] ["
                  << "] CP too large error " << "[" << act_cp(0) << "," << act_cp(1) << "] [m]" << std::endl;
      }
      initial_cp_too_large_error = false;
    } else {
      initial_cp_too_large_error = true;
    }
  }
  // tilt Check
  hrp::Vector3 fall_direction = hrp::Vector3::Zero();
  bool is_falling = false, will_fall = false;
  {
    double total_force = 0.0;
    for (size_t i = 0; i < stikp.size(); i++) {
      if (is_zmp_calc_enable[i]) {
        if (is_walking) {
          if (projected_normal.at(i).norm() > sin(tilt_margin[0])) {
            will_fall = true;
            if (m_will_fall_counter[i] % static_cast <int>(1.0/dt) == 0 ) { // once per 1.0[s]
              std::cerr << "[" << print_str << "] ["
                        << "] " << stikp[i].ee_name << " cannot support total weight, "
                        << "swgsuptime : " << controlSwingSupportTime[i] << ", state : " << ref_contact_states[i]
                        << ", otherwise robot will fall down toward " << "(" << projected_normal.at(i)(0) << "," << projected_normal.at(i)(1) << ") direction" << std::endl;
            }
            m_will_fall_counter[i]++;
          } else {
            m_will_fall_counter[i] = 0;
          }
        }
        fall_direction += projected_normal.at(i) * act_force.at(i).norm();
        total_force += act_force.at(i).norm();
      }
    }
    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
      fall_direction = fall_direction / total_force;
    } else {
      fall_direction = hrp::Vector3::Zero();
    }
    if (fall_direction.norm() > sin(tilt_margin[1])) {
      is_falling = true;
      if (m_is_falling_counter % static_cast <int>(0.2/dt) == 0) { // once per 0.2[s]
        std::cerr << "[" << print_str << "] ["
                  << "] robot is falling down toward " << "(" << fall_direction(0) << "," << fall_direction(1) << ") direction" << std::endl;
      }
      m_is_falling_counter++;
    } else {
      m_is_falling_counter = 0;
    }
  }
  // Total check for emergency signal
  switch (emergency_check_mode) {
  case OpenHRP::AutoBalancerService::NO_CHECK:
    is_emergency = false;
    break;
  case OpenHRP::AutoBalancerService::COP:
    is_emergency = is_cop_outside && is_seq_interpolating;
    break;
  case OpenHRP::AutoBalancerService::CP:
    is_emergency = is_cp_outside;
    break;
  case OpenHRP::AutoBalancerService::TILT:
    is_emergency = will_fall || is_falling;
    break;
  default:
    break;
  }
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] EmergencyCheck ("
              << (emergency_check_mode == OpenHRP::AutoBalancerService::NO_CHECK?"NO_CHECK": (emergency_check_mode == OpenHRP::AutoBalancerService::COP?"COP":"CP") )
              << ") " << (is_emergency?"emergency":"non-emergency") << std::endl;
  }
  rel_ee_pos.clear();
  rel_ee_rot.clear();
  rel_ee_name.clear();
};

void Stabilizer::calcSwingSupportLimbGain ()
{
  for (size_t i = 0; i < stikp.size(); i++) {
    STIKParam& ikp = stikp[i];
    if (ref_contact_states[i]) { // Support
      // Limit too large support time increment. Max time is 3600.0[s] = 1[h], this assumes that robot's one step time is smaller than 1[h].
      ikp.support_time = std::min(3600.0, ikp.support_time+dt);
      // In some PC, does not work because the first line is optimized out.
      // ikp.support_time += dt;
      // ikp.support_time = std::min(3600.0, ikp.support_time);
      if (ikp.support_time > eefm_pos_transition_time) {
        ikp.swing_support_gain = (controlSwingSupportTime[i] / eefm_pos_transition_time);
      } else {
        ikp.swing_support_gain = (ikp.support_time / eefm_pos_transition_time);
      }
      ikp.swing_support_gain = std::max(0.0, std::min(1.0, ikp.swing_support_gain));
    } else { // Swing
      ikp.swing_support_gain = 0.0;
      ikp.support_time = 0.0;
    }
  }
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] SwingSupportLimbGain = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << stikp[i].swing_support_gain << " ";
    std::cerr << "], ref_contact_states = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << ref_contact_states[i] << " ";
    std::cerr << "], sstime = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << controlSwingSupportTime[i] << " ";
    std::cerr << "], toeheel_ratio = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << toeheel_ratio[i] << " ";
    std::cerr << "], support_time = [";
    for (size_t i = 0; i < stikp.size(); i++) std::cerr << stikp[i].support_time << " ";
    std::cerr << "]" << std::endl;
  }
}

void Stabilizer::calcTPCC() {
  // stabilizer loop
  // Choi's feedback law
  hrp::Vector3 cog = m_robot->calcCM();
  hrp::Vector3 newcog = hrp::Vector3::Zero();
  hrp::Vector3 dcog(ref_cog - act_cog);
  hrp::Vector3 dzmp(ref_zmp - act_zmp);
  for (size_t i = 0; i < 2; i++) {
    double uu = ref_cogvel(i) - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
      + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
    newcog(i) = uu * dt + cog(i);
  }

  moveBasePosRotForBodyRPYControl ();

  // target at ee => target at link-origin
  hrp::Vector3 target_link_p[stikp.size()];
  hrp::Matrix33 target_link_R[stikp.size()];
  for (size_t i = 0; i < stikp.size(); i++) {
    rats::rotm3times(target_link_R[i], target_ee_R[i], stikp[i].localR.transpose());
    target_link_p[i] = target_ee_p[i] - target_ee_R[i] * stikp[i].localCOPPos;
  }
  // solveIK
  //   IK target is link origin pos and rot, not ee pos and rot.
  //for (size_t jj = 0; jj < 5; jj++) {
  size_t max_ik_loop_count = 0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (max_ik_loop_count < stikp[i].ik_loop_count) max_ik_loop_count = stikp[i].ik_loop_count;
  }
  for (size_t jj = 0; jj < max_ik_loop_count; jj++) {
    hrp::Vector3 tmpcm = m_robot->calcCM();
    for (size_t i = 0; i < 2; i++) {
      m_robot->rootLink()->p(i) = m_robot->rootLink()->p(i) + 0.9 * (newcog(i) - tmpcm(i));
    }
    m_robot->calcForwardKinematics();
    for (size_t i = 0; i < stikp.size(); i++) {
      if (is_ik_enable[i]) {
        jpe_v[i]->calcInverseKinematics2Loop(target_link_p[i], target_link_R[i], 1.0, stikp[i].avoid_gain, stikp[i].reference_gain, &qrefv, transition_smooth_gain);
      }
    }
  }
}

void Stabilizer::calcEEForceMomentControl()
{
  // stabilizer loop
  // return to referencea
  m_robot->rootLink()->R = target_root_R;
  m_robot->rootLink()->p = target_root_p;
  for ( int i = 0; i < m_robot->numJoints(); i++ ) {
    m_robot->joint(i)->q = qrefv[i];
  }
  for (size_t i = 0; i < jpe_v.size(); i++) {
    if (is_ik_enable[i]) {
      for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
        int idx = jpe_v[i]->joint(j)->jointId;
        m_robot->joint(idx)->q = qorg[idx];
      }
    }
  }
  // Fix for toe joint
  for (size_t i = 0; i < jpe_v.size(); i++) {
    if (is_ik_enable[i]) {
      if (jpe_v[i]->numJoints() == 7) {
        int idx = jpe_v[i]->joint(jpe_v[i]->numJoints() -1)->jointId;
        m_robot->joint(idx)->q = qrefv[idx];
      }
    }
  }

  // State calculation for swing ee compensation
  //   joint angle : current control output
  //   root pos : target root p
  //   root rot : actual root rot
  {
    // Calc status
    m_robot->rootLink()->R = target_root_R;
    m_robot->rootLink()->p = target_root_p;
    m_robot->calcForwardKinematics();
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(rpy));
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics();
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
    // Calculate foot_origin_coords-relative ee pos and rot
    // Subtract them from target_ee_diff_xx
    for (size_t i = 0; i < stikp.size(); i++) {
      hrp::Link* target = m_robot->link(stikp[i].target_name);
      stikp[i].target_ee_diff_p -= foot_origin_rot.transpose() * (target->p + target->R * stikp[i].localp - foot_origin_pos);
      stikp[i].target_ee_diff_r = (foot_origin_rot.transpose() * target->R * stikp[i].localR).transpose() * stikp[i].target_ee_diff_r;
    }
  }

  // State calculation for control : calculate "current" state
  //   joint angle : current control output
  //   root pos : target + keep COG against rpy control
  //   root rot : target + rpy control
  moveBasePosRotForBodyRPYControl ();

  // Convert d_foot_pos in foot origin frame => "current" world frame
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  std::vector<hrp::Vector3> current_d_foot_pos;
  for (size_t i = 0; i < stikp.size(); i++)
    current_d_foot_pos.push_back(foot_origin_rot * stikp[i].d_foot_pos);

  // Swing ee compensation.
  calcSwingEEModification();

  // solveIK
  //   IK target is link origin pos and rot, not ee pos and rot.
  std::vector<hrp::Vector3> tmpp(stikp.size());
  std::vector<hrp::Matrix33> tmpR(stikp.size());
  double tmp_d_pos_z_root = 0.0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (is_ik_enable[i]) {
      // Add damping_control compensation to target value
      if (is_feedback_control_enable[i]) {
        rats::rotm3times(tmpR[i], target_ee_R[i], hrp::rotFromRpy(-1*stikp[i].ee_d_foot_rpy));
        // foot force difference control version
        // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
        // foot force independent damping control
        tmpp[i] = target_ee_p[i] - current_d_foot_pos[i];
      } else {
        tmpp[i] = target_ee_p[i];
        tmpR[i] = target_ee_R[i];
      }
      // Add swing ee compensation
      rats::rotm3times(tmpR[i], tmpR[i], hrp::rotFromRpy(stikp[i].d_rpy_swing));
      tmpp[i] = tmpp[i] + foot_origin_rot * stikp[i].d_pos_swing;
    }
  }

  limbStretchAvoidanceControl(tmpp ,tmpR);

  // IK
  for (size_t i = 0; i < stikp.size(); i++) {
    if (is_ik_enable[i]) {
      for (size_t jj = 0; jj < stikp[i].ik_loop_count; jj++) {
        jpe_v[i]->calcInverseKinematics2Loop(tmpp[i], tmpR[i], 1.0, 0.001, 0.01, &qrefv, transition_smooth_gain,
                                             //stikp[i].localCOPPos;
                                             stikp[i].localp,
                                             stikp[i].localR);
      }
    }
  }
}

// Swing ee compensation.
//   Calculate compensation values to minimize the difference between "current" foot-origin-coords-relative pos and rot and "target" foot-origin-coords-relative pos and rot for swing ee.
//   Input  : target_ee_diff_p, target_ee_diff_r
//   Output : d_pos_swing, d_rpy_swing
void Stabilizer::calcSwingEEModification ()
{
  for (size_t i = 0; i < stikp.size(); i++) {
    // Calc compensation values
    double limit_pos = 30 * 1e-3; // 30[mm] limit
    double limit_rot = deg2rad(10); // 10[deg] limit
    if (ref_contact_states[contact_states_index_map[stikp[i].ee_name]] || act_contact_states[contact_states_index_map[stikp[i].ee_name]]) {
      // If actual contact or target contact is ON, do not use swing ee compensation. Exponential zero retrieving.
      stikp[i].d_rpy_swing = calcDampingControl(stikp[i].d_rpy_swing, stikp[i].eefm_swing_rot_time_const);
      stikp[i].d_pos_swing = calcDampingControl(stikp[i].d_pos_swing, stikp[i].eefm_swing_pos_time_const);
      stikp[i].target_ee_diff_p_filter->reset(stikp[i].d_pos_swing);
      stikp[i].target_ee_diff_r_filter->reset(stikp[i].d_rpy_swing);
    } else {
      /* position */
      {
        hrp::Vector3 tmpdiffp = stikp[i].eefm_swing_pos_spring_gain.cwiseProduct(stikp[i].target_ee_diff_p_filter->passFilter(stikp[i].target_ee_diff_p));
        double lvlimit = -50 * 1e-3 * dt, uvlimit = 50 * 1e-3 * dt; // 50 [mm/s]
        hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_pos_swing + lvlimit * hrp::Vector3::Ones();
        hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_pos_swing + uvlimit * hrp::Vector3::Ones();
        stikp[i].d_pos_swing = vlimit(vlimit(tmpdiffp, -1 * limit_pos, limit_pos), limit_by_lvlimit, limit_by_uvlimit);
      }
      /* rotation */
      {
        hrp::Vector3 tmpdiffr = stikp[i].eefm_swing_rot_spring_gain.cwiseProduct(stikp[i].target_ee_diff_r_filter->passFilter(hrp::rpyFromRot(stikp[i].target_ee_diff_r)));
        double lvlimit = deg2rad(-20.0*dt), uvlimit = deg2rad(20.0*dt); // 20 [deg/s]
        hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_rpy_swing + lvlimit * hrp::Vector3::Ones();
        hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_rpy_swing + uvlimit * hrp::Vector3::Ones();
        stikp[i].d_rpy_swing = vlimit(vlimit(tmpdiffr, -1 * limit_rot, limit_rot), limit_by_lvlimit, limit_by_uvlimit);
      }
    }
    stikp[i].prev_d_pos_swing = stikp[i].d_pos_swing;
    stikp[i].prev_d_rpy_swing = stikp[i].d_rpy_swing;
  }
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] Swing foot control" << std::endl;
    for (size_t i = 0; i < stikp.size(); i++) {
      std::cerr << "[" << print_str << "]   "
                << "d_rpy_swing (" << stikp[i].ee_name << ")  = " << (stikp[i].d_rpy_swing / M_PI * 180.0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[deg], "
                << "d_pos_swing (" << stikp[i].ee_name << ")  = " << (stikp[i].d_pos_swing * 1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm]" << std::endl;
    }
  }
};

void Stabilizer::limbStretchAvoidanceControl (const std::vector<hrp::Vector3>& ee_p, const std::vector<hrp::Matrix33>& ee_R)
{
  double tmp_d_pos_z_root = 0.0, prev_d_pos_z_root = d_pos_z_root;
  if (use_limb_stretch_avoidance) {
    for (size_t i = 0; i < stikp.size(); i++) {
      if (is_ik_enable[i]) {
        // Check whether inside limb length limitation
        hrp::Link* parent_link = m_robot->link(stikp[i].parent_name);
        hrp::Vector3 targetp = (ee_p[i] - ee_R[i] * stikp[i].localR.transpose() * stikp[i].localp) - parent_link->p; // position from parent to target link (world frame)
        double limb_length_limitation = stikp[i].max_limb_length - stikp[i].limb_length_margin;
        double tmp = limb_length_limitation * limb_length_limitation - targetp(0) * targetp(0) - targetp(1) * targetp(1);
        if (targetp.norm() > limb_length_limitation && tmp >= 0) {
          tmp_d_pos_z_root = std::min(tmp_d_pos_z_root, targetp(2) + std::sqrt(tmp));
        }
      }
    }
    // Change root link height depending on limb length
    d_pos_z_root = tmp_d_pos_z_root == 0.0 ? calcDampingControl(d_pos_z_root, limb_stretch_avoidance_time_const) : tmp_d_pos_z_root;
  } else {
    d_pos_z_root = calcDampingControl(d_pos_z_root, limb_stretch_avoidance_time_const);
  }
  d_pos_z_root = vlimit(d_pos_z_root, prev_d_pos_z_root + limb_stretch_avoidance_vlimit[0], prev_d_pos_z_root + limb_stretch_avoidance_vlimit[1]);
  m_robot->rootLink()->p(2) += d_pos_z_root;
}

void Stabilizer::setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name)
{
  std::vector<bool> prev_values;
  prev_values.resize(st_bool_values.size());
  copy (st_bool_values.begin(), st_bool_values.end(), prev_values.begin());
  if (st_bool_values.size() != output_bool_values.length()) {
    std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set. Length " << st_bool_values.size() << " != " << output_bool_values.length() << std::endl;
  } else if ( (control_mode != MODE_IDLE) ) {
    std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set. Current control_mode is " << control_mode << std::endl;
  } else {
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      st_bool_values[i] = output_bool_values[i];
    }
  }
  std::cerr << "[" << print_str << "]   " << prop_name << " is ";
  for (size_t i = 0; i < st_bool_values.size(); i++) {
    std::cerr <<"[" << st_bool_values[i] << "]";
  }
  std::cerr << "(set = ";
  for (size_t i = 0; i < output_bool_values.length(); i++) {
    std::cerr <<"[" << output_bool_values[i] << "]";
  }
  std::cerr << ", prev = ";
  for (size_t i = 0; i < prev_values.size(); i++) {
    std::cerr <<"[" << prev_values[i] << "]";
  }
  std::cerr << ")" << std::endl;
};

void Stabilizer::setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalancerService::BoolSequence& output_bool_values, const std::string& prop_name)
{
  std::vector<bool> prev_values;
  prev_values.resize(st_bool_values.size());
  copy (st_bool_values.begin(), st_bool_values.end(), prev_values.begin());
  if (st_bool_values.size() != output_bool_values.length()) {
    std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set. Length " << st_bool_values.size() << " != " << output_bool_values.length() << std::endl;
  } else if ( control_mode == MODE_IDLE ) {
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      st_bool_values[i] = output_bool_values[i];
    }
  } else {
    std::vector<size_t> failed_indices;
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      if ( (st_bool_values[i] != output_bool_values[i]) ) { // If mode change
        if (!ref_contact_states[i] ) { // reference contact_states should be OFF
          st_bool_values[i] = output_bool_values[i];
        } else {
          failed_indices.push_back(i);
        }
      }
    }
    if (failed_indices.size() > 0) {
      std::cerr << "[" << print_str << "]   " << prop_name << " cannot be set partially. failed_indices is [";
      for (size_t i = 0; i < failed_indices.size(); i++) {
        std::cerr << failed_indices[i] << " ";
      }
      std::cerr << "]" << std::endl;
    }
  }
  std::cerr << "[" << print_str << "]   " << prop_name << " is ";
  for (size_t i = 0; i < st_bool_values.size(); i++) {
    std::cerr <<"[" << st_bool_values[i] << "]";
  }
  std::cerr << "(set = ";
  for (size_t i = 0; i < output_bool_values.length(); i++) {
    std::cerr <<"[" << output_bool_values[i] << "]";
  }
  std::cerr << ", prev = ";
  for (size_t i = 0; i < prev_values.size(); i++) {
    std::cerr <<"[" << prev_values[i] << "]";
  }
  std::cerr << ")" << std::endl;
};

std::string Stabilizer::getStabilizerAlgorithmString (OpenHRP::AutoBalancerService::STAlgorithm _st_algorithm)
{
  switch (_st_algorithm) {
  case OpenHRP::AutoBalancerService::TPCC:
    return "TPCC";
  case OpenHRP::AutoBalancerService::EEFM:
    return "EEFM";
  case OpenHRP::AutoBalancerService::EEFMQP:
    return "EEFMQP";
  case OpenHRP::AutoBalancerService::EEFMQPCOP:
    return "EEFMQPCOP";
  case OpenHRP::AutoBalancerService::EEFMQPCOP2:
    return "EEFMQPCOP2";
  default:
    return "";
  }
};

void Stabilizer::calcDiffFootOriginExtMoment ()
{
  // calc reference ext moment around foot origin pos
  // static const double grav = 9.80665; /* [m/s^2] */
  double mg = total_mass * eefm_gravitational_acceleration;
  hrp::Vector3 ref_ext_moment = hrp::Vector3(mg * ref_cog(1) - ref_total_foot_origin_moment(0),
                                             -mg * ref_cog(0) - ref_total_foot_origin_moment(1),
                                             0);
  // calc act ext moment around foot origin pos
  hrp::Vector3 act_ext_moment = hrp::Vector3(mg * act_cog(1) - act_total_foot_origin_moment(0),
                                             -mg * act_cog(0) - act_total_foot_origin_moment(1),
                                             0);
  // Do not calculate actual value if in the air, because of invalid act_zmp.
  if ( !on_ground ) act_ext_moment = ref_ext_moment;
  // Calc diff
  diff_foot_origin_ext_moment = ref_ext_moment - act_ext_moment;
  if (DEBUGP) {
    std::cerr << "[" << print_str << "] DiffStaticBalancePointOffset" << std::endl;
    std::cerr << "[" << print_str << "]   "
              << "ref_ext_moment = " << ref_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm], "
              << "act_ext_moment = " << act_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm], "
              << "diff ext_moment = " << diff_foot_origin_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm]" << std::endl;
  }
};
