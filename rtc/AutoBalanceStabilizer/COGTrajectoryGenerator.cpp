// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#include "COGTrajectoryGenerator.h"

namespace hrp {

void COGTrajectoryGenerator::initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp)
{
    // std::cerr << "cog z: " << cog(2) << " ref cog z: " << cog(2) - cur_ref_zmp(2) << std::endl;
    preview_controller.reset(new ExtendedPreviewController(dt, cog(2) - cur_ref_zmp(2), cur_ref_zmp));
}

void COGTrajectoryGenerator::calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list)
{
    if (calculation_type == PREVIEW_CONTROL) {
        preview_controller->calc_x_k(refzmp_list);
        cog     = preview_controller->getRefCog();
        cog_vel = preview_controller->getRefCogVel();
        cog_acc = preview_controller->getRefCogAcc();
    }
}

void COGTrajectoryGenerator::calcCogFromLandingPoints(const hrp::Vector3& support_point,
                                                      const hrp::Vector3& landing_point,
                                                      const hrp::Vector3& start_zmp_offset,
                                                      const hrp::Vector3& end_zmp_offset,
                                                      const hrp::Vector3& target_cp_offset,
                                                      const double jump_height,
                                                      const double dt,
                                                      const double start_time,
                                                      const double supporting_time,
                                                      const double landing_time,
                                                      const double cur_time,
                                                      const bool is_first)
{
    const double g_acc = 9.80665;
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    double flight_time = 2 * take_off_z_vel / g_acc;

    const hrp::Vector3 target_cp = landing_point + target_cp_offset;
    const double cog_height = 0.8;
    const double rel_cur_time = cur_time - start_time;
    const double rel_land_time = landing_time - start_time;
    const double omega = std::sqrt(g_acc / cog_height);
    const double omega_tau = omega * supporting_time;

    const hrp::Vector3 cp = cog + cog_vel / omega;
    double omega_T = omega * flight_time;
    hrp::Vector3 c_1 = hrp::Vector3::Zero();
    hrp::Vector3 ref_zmp = hrp::Vector3::Zero();

    // 3次関数
    {
        const double tau = supporting_time;
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double omega2 = omega * omega;
        const double omega3 = omega2 * omega;

        hrp::Vector3 a_var = hrp::Vector3::Zero();
        hrp::Vector3 b_var = hrp::Vector3::Zero();
        hrp::Vector3 c_var = hrp::Vector3::Zero();
        hrp::Vector3 d_var = hrp::Vector3::Zero();

        {
            Eigen::Matrix<double, 6, 6> A;
            A <<
                0, 0, 0, 1, 0, 0,
                tau3, tau2, tau, 1, 0, 0,
                6 / omega3, 2 / omega2, 1 / omega, 1, 0, 2,
                (flight_time + 1 / omega) * (3 * tau2 + 6 / omega2), 2 * (flight_time + 1 / omega) * tau, flight_time + 1 / omega, 0, (omega_T + 1) * exp(omega * tau), -(omega_T + 1) * exp(-omega * tau),
                0, 2 / omega2, 0, 0, 1, 1,
                6 * tau / omega2, 2 / omega2, 0, 0, exp(omega * tau), exp(-omega * tau);

            const auto A_lu = A.partialPivLu();
            for (size_t i = 0; i < 2; ++i) {
                Eigen::Matrix<double, 6, 1> B;
                B << support_point[i] + start_zmp_offset[i], support_point[i] + end_zmp_offset[i], support_point[i], target_cp[i] - (support_point[i] + end_zmp_offset[i]), 0, 0;
                const Eigen::Matrix<double, 6, 1> ans = A_lu.solve(B);

                a_var[i] = ans[0];
                b_var[i] = ans[1];
                c_var[i] = ans[2];
                d_var[i] = ans[3];
            }
        }

        const auto calcA = [&](const double t) { return d_var + (t + 1 / omega) * c_var + (t * t + 2 * t / omega + 2 / omega2) * b_var + (t * t * t + 3 * t * t / omega + 6 * t / omega2 + 6 / omega3) * a_var; };
        const hrp::Vector3 B_tau = c_var + 2 * b_var * (tau + 1 / omega) + 3 * a_var * (tau * tau + 2 * tau / omega + 2 / omega2);

        c_1 = (cp - calcA(rel_cur_time) - (target_cp - (calcA(tau) + B_tau * flight_time)) / (omega_T + 1) * std::exp(-omega * (tau - rel_cur_time))) / (-(omega * tau * flight_time + flight_time + tau) / (omega_T + 1) * omega3 * std::exp(omega * rel_cur_time) - (omega2 * flight_time * flight_time + omega * flight_time - 2) / (2 * flight_time * (omega_T + 1)) * std::exp(omega * (2 * rel_land_time - 2 * tau + rel_cur_time)) + omega3 * rel_cur_time * std::exp(omega * rel_cur_time) - (omega2 * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time - rel_cur_time)));

        ref_zmp = a_var * rel_cur_time * rel_cur_time * rel_cur_time + b_var * rel_cur_time * rel_cur_time + c_var * rel_cur_time + d_var;
    }

    const hrp::Vector3 lambda = -(std::exp(omega * rel_cur_time) + (omega_T + 2) / (omega_T) * std::exp(omega * (2 * rel_land_time - rel_cur_time))) * c_1;
    hrp::Vector3 input_zmp = ref_zmp + omega * omega * lambda;

    constexpr double EPS = 1e-6;
    if (supporting_time - rel_cur_time > EPS) {
        // Z: 6th order function
        const double take_off_z = 0.85; // tmp
        const double rel_takeoff_time = supporting_time - rel_cur_time;
        const double T2 = rel_takeoff_time * rel_takeoff_time;
        const double T3 = T2 * rel_takeoff_time;
        const double T4 = T3 * rel_takeoff_time;
        const double T5 = T4 * rel_takeoff_time;

        const double a = (-T2 * (cog_acc[2] + g_acc) - 6 * rel_takeoff_time * (cog_vel[2] + take_off_z_vel) + 12 * (-cog[2] + take_off_z)) / (2 * T5);
        const double b = (T2 * (3 * cog_acc[2] + 2 * g_acc) + 2 * rel_takeoff_time * (8 * cog_vel[2] + 7 * take_off_z_vel) + 30 * (cog[2] - take_off_z)) / (2 * T4);
        const double c = (-T2 * (3 * cog_acc[2] + g_acc) - 4 * rel_takeoff_time * (3 * cog_vel[2] + 2 * take_off_z_vel) + 20 * (-cog[2] + take_off_z)) / (2 * T3);
        const double d = cog_acc[2] / 2;
        const double e = cog_vel[2];
        const double f = cog[2];

        cog_acc.head<2>() = (omega * omega * (cog - input_zmp)).head<2>();
        cog += cog_vel * dt + cog_acc * dt * dt * 0.5;
        cog_vel += cog_acc * dt;

        cog[2]     = a * dt*dt*dt*dt*dt  + b * dt*dt*dt*dt  + c * dt*dt*dt  + d * dt*dt  + e * dt + f;
        cog_vel[2] = 5 * a * dt*dt*dt*dt + 4 * b * dt*dt*dt + 3 * c * dt*dt + 2 * d * dt + e;
        cog_acc[2] = 20 * a * dt*dt*dt   + 12 * b * dt*dt   + 6 * c * dt    + 2 * d;
    } else { // Flight phase
        cog_acc = hrp::Vector3(0, 0, -g_acc);
        input_zmp.setZero();
        ref_zmp.setZero(); // Debug
        c_1.setZero();

        cog += cog_vel * dt + cog_acc * dt * dt * 0.5;
        cog_vel += cog_acc * dt;
    }

    // cog += cog_vel * dt + cog_acc * dt * dt * 0.5;
    // cog_vel += cog_acc * dt;
}

}
