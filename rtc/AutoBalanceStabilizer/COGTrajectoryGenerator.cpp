// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#include <array>
#include "Utility.h"
#include "TDMASolver.h"
#include "LinkConstraint.h"
#include "COGTrajectoryGenerator.h"

namespace hrp {

void COGTrajectoryGenerator::initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp) // TODO: dt後にする?
{
    // std::cerr << "cog z: " << cog(2) << " ref cog z: " << cog(2) - cur_ref_zmp(2) << std::endl;
    preview_controller.reset(new ExtendedPreviewController(dt, cog(2) - cur_ref_zmp(2), cur_ref_zmp));
}

void COGTrajectoryGenerator::calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list, const double dt)
{
    if (calculation_type == PREVIEW_CONTROL) {
        preview_controller->calc_x_k(refzmp_list);
        // tmp
        cog.head<2>()     = preview_controller->getRefCog().head<2>();
        cog_vel.head<2>() = preview_controller->getRefCogVel().head<2>();
        cog_acc.head<2>() = preview_controller->getRefCogAcc().head<2>();
    } else {
        cog_acc.head<2>()  = (omega * omega * (cog - refzmp_list[0])).head<2>();
        cog               += cog_vel * dt + cog_acc * dt * dt * 0.5;
        cog_vel           += cog_acc * dt;
    }
}

hrp::Vector3 COGTrajectoryGenerator::calcCogForFlightPhase(const double dt, const double g_acc)
{
    cog_acc = hrp::Vector3(0, 0, -g_acc);
    cog += cog_vel * dt + cog_acc * dt * dt * 0.5;
    cog_vel += cog_acc * dt;
    return hrp::Vector3::Zero();
 }

// 使わない？
hrp::Vector3 COGTrajectoryGenerator::calcCogForRun(const hrp::Vector3& support_point,
                                                   const hrp::Vector3& landing_point,
                                                   const hrp::Vector3& start_zmp_offset,
                                                   const hrp::Vector3& end_zmp_offset,
                                                   const hrp::Vector3& target_cp_offset,
                                                   const double take_off_z,
                                                   const double jump_height,
                                                   const size_t start_count,
                                                   const size_t supporting_count,
                                                   const size_t landing_count,
                                                   const size_t cur_count,
                                                   const double dt,
                                                   const FootGuidedRefZMPType ref_zmp_type,
                                                   const double g_acc)
{
    if (cur_count < start_count) return hrp::Vector3::Zero();

    const size_t rel_cur_count = cur_count - start_count;
    if (supporting_count <= rel_cur_count) return calcCogForFlightPhase(dt, g_acc);

    const hrp::Vector3 ref_zmp = calcFootGuidedCog(support_point, landing_point, start_zmp_offset, end_zmp_offset,
                                                   target_cp_offset, jump_height, start_count, supporting_count,
                                                   landing_count, cur_count, dt, ref_zmp_type, g_acc);
    calcCogZForJump(supporting_count - rel_cur_count, jump_height, take_off_z, dt, g_acc);
    return ref_zmp;
}

void COGTrajectoryGenerator::calcCogZForJump(const size_t count_to_jump,
                                             const double jump_height,
                                             const double take_off_z,
                                             const double dt,
                                             const double g_acc)
{
    // 6th order function: s.t. cog_acc(T) = -g_acc
    // std::cerr << "cogz: " << cog[2] << ", cog_velz: " << cog_vel[2] << ", cog_accz: " << cog_acc[2] << std::endl;
    // std::cerr << "height: " << jump_height << ", take_off_z: " << take_off_z << ", dt: " << dt << ", g_acc: " << g_acc << std::endl;
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    const double T  = count_to_jump * dt;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    const double a = (-T2 * (cog_acc[2] + g_acc) - 6 * T * (cog_vel[2] + take_off_z_vel) + 12 * (-cog[2] + take_off_z)) / (2 * T5);
    const double b = (T2 * (3 * cog_acc[2] + 2 * g_acc) + 2 * T * (8 * cog_vel[2] + 7 * take_off_z_vel) + 30 * (cog[2] - take_off_z)) / (2 * T4);
    const double c = (-T2 * (3 * cog_acc[2] + g_acc) - 4 * T * (3 * cog_vel[2] + 2 * take_off_z_vel) + 20 * (-cog[2] + take_off_z)) / (2 * T3);
    const double d = cog_acc[2] / 2;
    const double e = cog_vel[2];
    const double f = cog[2];

    cog[2]     = a * dt*dt*dt*dt*dt  + b * dt*dt*dt*dt  + c * dt*dt*dt  + d * dt*dt  + e * dt + f;
    cog_vel[2] = 5 * a * dt*dt*dt*dt + 4 * b * dt*dt*dt + 3 * c * dt*dt + 2 * d * dt + e;
    cog_acc[2] = 20 * a * dt*dt*dt   + 12 * b * dt*dt   + 6 * c * dt    + 2 * d;

    diff_ref_cog_z = ref_cog_z - cog[2];
}

std::vector<std::tuple<double, double, double>> COGTrajectoryGenerator::calcCogZListForJump(const size_t count_to_jump,
                                                                                            const double jump_height,
                                                                                            const double take_off_z,
                                                                                            const double dt,
                                                                                            const double g_acc)
{
    // 6th order function: s.t. cog_acc(T) = -g_acc
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    const double T  = count_to_jump * dt;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    const double a = (-T2 * (cog_acc[2] + g_acc) - 6 * T * (cog_vel[2] + take_off_z_vel) + 12 * (-cog[2] + take_off_z)) / (2 * T5);
    const double b = (T2 * (3 * cog_acc[2] + 2 * g_acc) + 2 * T * (8 * cog_vel[2] + 7 * take_off_z_vel) + 30 * (cog[2] - take_off_z)) / (2 * T4);
    const double c = (-T2 * (3 * cog_acc[2] + g_acc) - 4 * T * (3 * cog_vel[2] + 2 * take_off_z_vel) + 20 * (-cog[2] + take_off_z)) / (2 * T3);
    const double d = cog_acc[2] / 2;
    const double e = cog_vel[2];
    const double f = cog[2];

    std::vector<std::tuple<double, double, double>> cog_z_list;
    cog_z_list.reserve(count_to_jump);
    for (size_t i = 1; i <= count_to_jump; ++i) {
        const double t = dt * i;
        const double cog     = a * t*t*t*t*t   + b * t*t*t*t   + c * t*t*t   + d * t*t   + e * t + f;
        const double cog_vel = 5 * a * t*t*t*t + 4 * b * t*t*t + 3 * c * t*t + 2 * d * t + e;
        const double cog_acc = 20 * a * t*t*t  + 12 * b * t*t  + 6 * c * t   + 2 * d;
        cog_z_list.emplace_back(cog, cog_vel, cog_acc);
    }

    return cog_z_list;
}

void COGTrajectoryGenerator::calcCogListForRun(const hrp::Vector3 target_cp,
                                               const hrp::Vector3 ref_zmp,
                                               const size_t count_to_jump,
                                               const size_t cur_count,
                                               const double jump_height,
                                               const double take_off_z,
                                               const double dt,
                                               const double g_acc)
{
    cog_list_start_count = cur_count;
    const std::vector<std::tuple<double, double, double>>
        ref_cog_z_list = calcCogZListForJump(count_to_jump, jump_height, take_off_z, dt, g_acc);

    const size_t dim = count_to_jump;
    std::vector<double> a(dim);
    std::vector<double> b(dim);
    std::vector<double> c(dim);

    const double dt2 = dt * dt;
    for (size_t i = 0; i < dim; ++i) {
        const double denominator = (std::get<2>(ref_cog_z_list[i]) + g_acc) * dt2;
        const double cog_z = std::get<0>(ref_cog_z_list[i]);

        a[i] = -cog_z / denominator;
        b[i] = 2 * cog_z / denominator + 1;
        c[i] = a[i];
    }

    const double a_zero = a[0];
    a[0] = 0;
    b[0] += a_zero * (1 + std::sqrt(g_acc / std::get<0>(ref_cog_z_list[0])) * dt);
    c.back() = 0;

    // Capture point
    {
        const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
        const double flight_time = 2 * take_off_z_vel / g_acc;
        const double taking_off_omega = std::sqrt(g_acc / std::get<0>(ref_cog_z_list.back())); // TODO: 加速度考慮したい
        a.back() = -(flight_time * taking_off_omega + 1) / (taking_off_omega * dt);
        b.back() = 1 - a.back();
    }

    const hrp::Vector3 cur_cp = calcCP();
    cog_list.resize(dim);
    for (size_t xy = 0; xy <= 1; ++xy) {
        std::vector<double> ref_list(dim, ref_zmp[xy]);
        ref_list[0] += a_zero * omega * cur_cp[xy] * dt;
        ref_list.back() = target_cp[xy];

        const std::vector<double> ref_cog_list = (xy == 0) ? solveTDMAPreserve(a, b, c, ref_list) : solveTDMA(a, b, c, ref_list);
        for (size_t i = 0; i < dim; ++i) cog_list[i][xy] = ref_cog_list[i];
    }

    for (size_t i = 0; i < dim; ++i) cog_list[i][2] = std::get<0>(ref_cog_z_list[i]);
}

hrp::Vector3 COGTrajectoryGenerator::calcFootGuidedCog(const hrp::Vector3& support_point,
                                                       const hrp::Vector3& landing_point,
                                                       const hrp::Vector3& start_zmp_offset,
                                                       const hrp::Vector3& end_zmp_offset,
                                                       const hrp::Vector3& target_cp_offset,
                                                       const double jump_height,
                                                       const size_t start_count,
                                                       const size_t supporting_count,
                                                       const size_t landing_count,
                                                       const size_t cur_count,
                                                       const double dt,
                                                       const FootGuidedRefZMPType ref_zmp_type,
                                                       const double g_acc)
{
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    const double flight_time = 2 * take_off_z_vel / g_acc;

    const hrp::Vector3 target_cp = landing_point + target_cp_offset;
    const double rel_cur_time = (cur_count - start_count) * dt;
    const double rel_land_time = (landing_count - start_count) * dt;

    const hrp::Vector3 cp = cog + cog_vel / omega;
    const double omega_T = omega * flight_time;
    hrp::Vector3 c_1 = hrp::Vector3::Zero();
    hrp::Vector3 ref_zmp = support_point;
    hrp::Vector3 input_zmp = hrp::Vector3::Zero();

    hrp::Vector3 A_t;
    hrp::Vector3 Atau_dAtauT;

    if (ref_zmp_type == FIX) {
        A_t         = support_point;
        Atau_dAtauT = support_point;
        ref_zmp     = support_point;
    } else if (ref_zmp_type == CUBIC) {
        const double tau = supporting_count * dt;
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
                (flight_time + 1 / omega) * (3 * tau2 + 6 / omega2), 2 * (flight_time + 1 / omega) * tau, flight_time + 1 / omega, 0, (omega_T + 1) * std::exp(omega * tau), -(omega_T + 1) * std::exp(-omega * tau),
                0, 2 / omega2, 0, 0, 1, 1,
                6 * tau / omega2, 2 / omega2, 0, 0, std::exp(omega * tau), std::exp(-omega * tau);

            const auto A_lu = A.partialPivLu();
            for (size_t i = 0; i < 2; ++i) {
                Eigen::Matrix<double, 6, 1> B;
                B <<
                    support_point[i] + start_zmp_offset[i],
                    support_point[i] + end_zmp_offset[i],
                    support_point[i],
                    target_cp[i] - (support_point[i] + end_zmp_offset[i]),
                    0,
                    0;
                const Eigen::Matrix<double, 6, 1> ans = A_lu.solve(B);

                a_var[i] = ans[0];
                b_var[i] = ans[1];
                c_var[i] = ans[2];
                d_var[i] = ans[3];
            }
        }

        const auto calcA = [&](const double t) { return d_var + (t + 1 / omega) * c_var + (t * t + 2 * t / omega + 2 / omega2) * b_var + (t * t * t + 3 * t * t / omega + 6 * t / omega2 + 6 / omega3) * a_var; };

        const double supporting_time = supporting_count * dt;
        A_t = calcA(rel_cur_time);
        Atau_dAtauT = (c_var + 2 * b_var * (tau + 1 / omega) + 3 * a_var * (tau2 + 2 * tau / omega + 2 / omega2)) * flight_time;
        Atau_dAtauT += calcA(supporting_time);

        ref_zmp = a_var * rel_cur_time * rel_cur_time * rel_cur_time + b_var * rel_cur_time * rel_cur_time + c_var * rel_cur_time + d_var;
    }

    {
        const size_t rel_cur_count = cur_count - start_count;
        const double time_to_flight = (supporting_count - rel_cur_count) * dt;
        const double time_to_land = (landing_count - cur_count) * dt;
        const double omega2 = omega * omega;

        const double C_t = -2 * flight_time * time_to_flight * omega2 / (omega_T + 2) - 2 * flight_time * flight_time * omega2 / ((omega_T + 2) * (omega_T + 1)) - std::exp(2 * omega * time_to_land) - (omega_T - 1) * std::exp(2 * omega_T) / (omega_T + 1);
        const hrp::Vector3 D_t = cp - A_t - (landing_point - Atau_dAtauT) / (omega_T + 1) * std::exp(-omega * time_to_flight);
        const hrp::Vector3 omega2_lambda = -(2 * omega2 * flight_time / (omega_T + 2) + 2 * std::exp(2 * omega * time_to_land)) * D_t / C_t;

        input_zmp = ref_zmp + omega2_lambda;
    }

    // hrp::Vector3 input_zmp = ref_zmp + omega * omega * lambda;
    // const hrp::Vector3 min_zmp = support_point + hrp::Vector3(-0.06, -0.5, 0);
    // const hrp::Vector3 max_zmp = support_point + hrp::Vector3(0.16, 0.5, 0);
    // input_zmp = hrp::clamp(input_zmp, min_zmp, max_zmp);

    cog_acc.head<2>() = (omega * omega * (cog - input_zmp)).head<2>();
    cog.head<2>() += cog_vel.head<2>() * dt + cog_acc.head<2>() * dt * dt * 0.5;
    cog_vel.head<2>() += cog_acc.head<2>() * dt;

    return input_zmp;
    // return ref_zmp;
}

hrp::Vector3 COGTrajectoryGenerator::calcFootGuidedCogWalk(const std::vector<ConstraintsWithCount>& constraints_list,
                                                           const std::vector<std::pair<hrp::Vector3, size_t>>& ref_zmp_goals,
                                                           const int cur_const_idx,
                                                           const size_t cur_count,
                                                           const double dt,
                                                           const hrp::Vector3& target_cp_offset)
{
    const int landing_idx = getNextStableConstraints(constraints_list, cur_const_idx);
    if (landing_idx == -1) {
        std::cerr << "Cannot find next stable constraints and current constraints are also unstable. Something wrong!!" << std::endl;
        return hrp::Vector3::Zero();
    }

    const auto landing_point = constraints_list[landing_idx].calcCOPFromConstraints();
    hrp::Vector3 C2 = hrp::Vector3::Zero();
    hrp::Vector3 ref_zmp_a = hrp::Vector3::Zero();
    hrp::Vector3 ref_zmp_b = hrp::Vector3::Zero();

    double rel_cur_time = 0;
    double rel_goal_time;

    if (landing_idx == cur_const_idx) {
        constexpr double PREVIEW_TIME = 1.6;
        rel_goal_time = rel_cur_time + PREVIEW_TIME;
        C2 = (landing_point - constraints_list[landing_idx].calcStartCOPFromConstraints()) * std::exp(-omega * rel_goal_time);
        ref_zmp_a = landing_point;
    } else {
        size_t cur_zmp_idx = 0;
        const size_t zmp_goals_size = ref_zmp_goals.size();
        for (size_t idx = 1; idx < zmp_goals_size && ref_zmp_goals[idx].second <= cur_count; ++idx) {
            cur_zmp_idx = idx;
        }

        const double start_count = ref_zmp_goals[cur_zmp_idx].second;
        rel_cur_time = (cur_count - start_count) * dt;
        rel_goal_time = (ref_zmp_goals.back().second - start_count) * dt;

        {
            const ConstraintsWithCount& landing_constraints = constraints_list[landing_idx];
            const ConstraintsWithCount& prev_land_constraints = constraints_list[landing_idx - 1];
            const hrp::Vector3 landing_start_cop = landing_constraints.calcStartCOPFromConstraints();
            C2 = (landing_point - landing_start_cop) * std::exp(-omega * rel_goal_time) - (landing_start_cop - prev_land_constraints.calcStartCOPFromConstraints()) / ((landing_constraints.start_count - prev_land_constraints.start_count) * dt * omega) * std::exp(-omega * rel_goal_time);
        }

        for (size_t idx = cur_zmp_idx; idx < zmp_goals_size - 2; ++idx) {
            const double tn0 = (ref_zmp_goals[idx].second - start_count) * dt;
            const double tn1 = (ref_zmp_goals[idx + 1].second - start_count) * dt;
            const double tn2 = (ref_zmp_goals[idx + 2].second - start_count) * dt;
            C2 += 1 / omega * ((ref_zmp_goals[idx + 2].first - ref_zmp_goals[idx + 1].first) / (tn2 - tn1) - (ref_zmp_goals[idx + 1].first - ref_zmp_goals[idx].first) / (tn1 - tn0)) * std::exp(-omega * tn1);
        }

        ref_zmp_a = ref_zmp_goals[cur_zmp_idx].first;
        // const size_t next_zmp_idx = std::max(cur_zmp_idx + 1, zmp_goals_size - 1);
        ref_zmp_b = (ref_zmp_goals[cur_zmp_idx + 1].first - ref_zmp_goals[cur_zmp_idx].first) / ((ref_zmp_goals[cur_zmp_idx + 1].second - ref_zmp_goals[cur_zmp_idx].second) * dt);

        // const size_t next_idx = std::max(static_cast<int>(cur_const_idx + 1), landing_idx);
        // const double diff_time = (constraints_list[next_idx].start_count - constraints_list[cur_const_idx].start_count) * dt;
        // const hrp::Vector3 cur_start_cop  = constraints_list[cur_const_idx].calcStartCOPFromConstraints();
        // const hrp::Vector3 next_start_cop = constraints_list[next_idx].calcStartCOPFromConstraints();
        // // ref_zmp_a = (cur_start_cop * constraints_list[next_idx].start_count * dt - next_start_cop * constraints_list[cur_const_idx].start_count * dt) / diff_time;
        // ref_zmp_a = cur_start_cop;
        // ref_zmp_b = (next_start_cop - cur_start_cop) / diff_time;
    }

    const hrp::Vector3 ref_zmp = ref_zmp_a + ref_zmp_b * rel_cur_time;
    const hrp::Vector3 C1 = 2 * (calcCP() - C2 * std::exp(omega * rel_cur_time) - ref_zmp - ref_zmp_b / omega) / (1 - std::exp(-2 * omega * (rel_goal_time - rel_cur_time)));

    // std::cerr << "rel_cur_time: " << rel_cur_time << std::endl;
    // std::cerr << std::exp(omega * rel_cur_time) << std::endl;
    // std::cerr << "c2: " << C2.transpose() << std::endl;
    // std::cerr << "cp: " << calcCP().transpose() << std::endl;
    // // std::cerr << "omega: " << omega << ", (rel_goal_time - rel_cur_time): " << (rel_goal_time - rel_cur_time) << std::endl;
    // std::cerr << "ref_zmp: " << ref_zmp.transpose() << std::endl;
    // std::cerr << "c1: " << C1.transpose() << std::endl;

    const hrp::Vector3 input_cmp = ref_zmp + C1;
    cog_acc.head<2>() = (omega * omega * (cog - input_cmp)).head<2>();
    cog.head<2>() += cog_vel.head<2>() * dt + cog_acc.head<2>() * dt * dt * 0.5;
    cog_vel.head<2>() += cog_acc.head<2>() * dt;

    return input_cmp;
    // return ref_zmp;
}

hrp::Vector3 COGTrajectoryGenerator::calcFootGuidedCogWalk(const std::vector<ConstraintsWithCount>& constraints_list,
                                                           const hrp::Vector3& ref_zmp,
                                                           const hrp::Vector3& ref_zmp_vel,
                                                           const int cur_const_idx,
                                                           const size_t cur_count,
                                                           const double dt,
                                                           const hrp::Vector3& target_cp_offset)
{
    const int landing_idx = getNextStableConstraints(constraints_list, cur_const_idx);
    if (landing_idx == -1) {
        std::cerr << "Cannot find next stable constraints and current constraints are also unstable. Something wrong!!" << std::endl;
        return hrp::Vector3::Zero();
    }

    const auto landing_point = constraints_list[landing_idx].calcCOPFromConstraints();
    hrp::Vector3 C2 = hrp::Vector3::Zero();
    // hrp::Vector3 ref_zmp_a = hrp::Vector3::Zero();
    // hrp::Vector3 ref_zmp_b = hrp::Vector3::Zero();

    const double rel_cur_time = (cur_count - constraints_list[cur_const_idx].start_count) * dt;
    double rel_goal_time;

    if (landing_idx == cur_const_idx) {
        constexpr double PREVIEW_TIME = 1.6;
        rel_goal_time = rel_cur_time + PREVIEW_TIME;
        C2 = (landing_point - constraints_list[landing_idx].calcStartCOPFromConstraints()) * std::exp(-omega * rel_goal_time);
        // ref_zmp_a = landing_point;
    } else {
        const size_t cur_const_count = constraints_list[cur_const_idx].start_count;
        const ConstraintsWithCount& landing_constraints = constraints_list[landing_idx];
        {
            rel_goal_time = (landing_constraints.start_count - cur_const_count) * dt;
            const ConstraintsWithCount& prev_land_constraints = constraints_list[landing_idx - 1];
            const hrp::Vector3 landing_start_cop = landing_constraints.calcStartCOPFromConstraints();
            C2 = (landing_point - landing_start_cop) * std::exp(-omega * rel_goal_time) - (landing_start_cop - prev_land_constraints.calcStartCOPFromConstraints()) / ((landing_constraints.start_count - prev_land_constraints.start_count) * dt * omega) * std::exp(-omega * rel_goal_time);
        }

        // TODO: 両脚指示期間への切り替わりで，この辺がバグってそう
        for (int idx = landing_idx - 2; idx >= cur_const_idx; --idx) {
            const ConstraintsWithCount& n2_constraints = constraints_list[idx + 2];
            const ConstraintsWithCount& n1_constraints = constraints_list[idx + 1];
            const ConstraintsWithCount& n0_constraints = constraints_list[idx];
            const double tn2 = (n2_constraints.start_count - cur_const_count) * dt;
            const double tn1 = (n1_constraints.start_count - cur_const_count) * dt;
            const double tn0 = (n0_constraints.start_count - cur_const_count) * dt;

            const hrp::Vector3 n1_start_cop = n1_constraints.calcStartCOPFromConstraints();
            C2 += 1 / omega * ((n2_constraints.calcStartCOPFromConstraints() - n1_start_cop) / (tn2 - tn1) - (n1_start_cop - n0_constraints.calcStartCOPFromConstraints()) / (tn1 - tn0)) * std::exp(-omega * tn1);
        }

        const size_t next_idx = std::max(static_cast<int>(cur_const_idx + 1), landing_idx);
        const double diff_time = (constraints_list[next_idx].start_count - constraints_list[cur_const_idx].start_count) * dt;
        const hrp::Vector3 cur_start_cop  = constraints_list[cur_const_idx].calcStartCOPFromConstraints();
        const hrp::Vector3 next_start_cop = constraints_list[next_idx].calcStartCOPFromConstraints();
    }

    // const hrp::Vector3 ref_zmp = ref_zmp_a + ref_zmp_b * rel_cur_time;
    // TODO: まとめるかしないとrel_cur_timeが進むにつれ指数関数部分がオーバーフローしてnanになる
    const hrp::Vector3 C1 = 2 * (calcCP() - C2 * std::exp(omega * rel_cur_time) - ref_zmp - ref_zmp_vel / omega) / (1 - std::exp(-2 * omega * (rel_goal_time - rel_cur_time)));

    // std::cerr << "rel_cur_time: " << rel_cur_time << std::endl;
    // std::cerr << std::exp(omega * rel_cur_time) << std::endl;
    // std::cerr << "c2: " << C2.transpose() << std::endl;
    // std::cerr << "cp: " << calcCP().transpose() << std::endl;
    // // std::cerr << "omega: " << omega << ", (rel_goal_time - rel_cur_time): " << (rel_goal_time - rel_cur_time) << std::endl;
    // std::cerr << "ref_zmp: " << ref_zmp.transpose() << std::endl;
    // std::cerr << "c1: " << C1.transpose() << std::endl;

    const hrp::Vector3 input_cmp = ref_zmp + C1;
    cog_acc.head<2>() = (omega * omega * (cog - input_cmp)).head<2>();
    cog.head<2>() += cog_vel.head<2>() * dt + cog_acc.head<2>() * dt * dt * 0.5;
    cog_vel.head<2>() += cog_acc.head<2>() * dt;

    return input_cmp;
    // return ref_zmp;
}

}
