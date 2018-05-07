/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "FootGuidedController.h"


void foot_guided_control_base::set_mat(const double dz)
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

void foot_guided_control_base::calc_u(const std::size_t N, const double ref_dcm, const double ref_vrp)
{
    if ( N > 0 ) {
        double hn = std::pow(h, N);
        double hgain =  (hn / h) * (1 + h) / ( 1 - hn * hn );
        w_k = Phi * x_k;
        u_k = ref_vrp - hgain * (hn * (w_k(0) - ref_vrp + w_k_offset) - (ref_dcm - ref_vrp));
    } else {
        u_k = ref_vrp;
    }
    truncate_u();
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

void foot_guided_control_base::update_control(double& vrp, const std::size_t N, const double ref_dcm, const double ref_vrp)
{
    calc_u(N, ref_dcm, ref_vrp);
    vrp = u_k;
}

void foot_guided_control_base::update_state(double& pos)
{
    calc_x_k();
    pos = x_k(0);
}

void foot_guided_control_base::update(double& vrp, double& pos, const std::size_t N, const double ref_dcm, const double ref_vrp)
{
    update_control(vrp, N, ref_dcm, ref_vrp);
    update_state(pos);
}
