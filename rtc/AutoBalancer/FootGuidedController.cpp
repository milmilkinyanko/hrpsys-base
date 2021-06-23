/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "FootGuidedController.h"


void foot_guided_control_base::set_mat()
{
  xi = std::sqrt(g / dz);
  h = 1 + xi * dt;
  h_ = 1 - xi * dt;
  A <<
    1.0, dt,
    xi * xi * dt, 1.0;
  b <<
    0.0,
    -xi * xi * dt;
  Phi <<
    1.0, 1.0 / xi,
    1.0, -1.0 / xi;
  Phi_inv <<
    1.0, 1.0,
    xi, -xi;
  Phi_inv = 2.0 * Phi_inv;
}

double foot_guided_control_base::calc_u(const std::vector<LinearTrajectory<double>>& ref_zmp, const double cur_cp)
{
  double u = cur_cp - ref_zmp.front().getStart() - ref_zmp.front().getSlope() / xi; // 数式上の j = 0 に相当
  double Tj = 0.0;
  for (size_t j = 0; j < ref_zmp.size() - 1; j++) { // 数式上のindexとは1ずれてる
    Tj += ref_zmp.at(j).getTime();
    u += exp(- xi * Tj) * (ref_zmp.at(j).getGoal() - ref_zmp.at(j+1).getStart() + (ref_zmp.at(j).getSlope() - ref_zmp.at(j+1).getSlope()) / xi);
  }
  u = ref_zmp.front().getStart() + 2 * u / (1 - exp(-2 * xi * Tj));

  return u;
}

void foot_guided_control_base::truncate_u()
{
  double acc;
  get_acc(acc);
  if (std::abs(acc) > mu * g) {
    acc = mu * g * sgn(acc);
    u_k = x_k(0) - acc / (xi * xi);
  }
}

// assumed after calc_u
void foot_guided_control_base::calc_x_k()
{
  x_k = A * x_k + b * u_k;
}

void foot_guided_control_base::update_control(double& zmp, double& feedforward_zmp, const std::vector<LinearTrajectory<double>>& ref_zmp)
{
  u_k = calc_u(ref_zmp, (Phi * x_k)(0));
  act_u_k = calc_u(ref_zmp, (Phi * act_x_k)(0));

  feedforward_zmp = u_k;
  zmp = act_u_k;
}

void foot_guided_control_base::update_state(double& pos, const double fx)
{
  calc_x_k();
  dc_off = - fx / (xi * xi * mass); // 反力
  pos = x_k(0) + dc_off;
}

// void foot_guided_control_base::update(double& zmp, double& pos, const std::size_t N, const double ref_dcm, const double ref_zmp)
// {
//   update_control(zmp, N, ref_dcm, ref_zmp);
//   update_state(pos);
// }
