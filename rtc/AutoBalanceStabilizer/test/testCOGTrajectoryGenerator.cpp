// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testCOGTrajectoryGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <cstdio>
#include <fstream>
#include "../COGTrajectoryGenerator.h"

#include <chrono>

namespace
{
constexpr double dt = 0.002;
constexpr double max_time = 8.0;
constexpr size_t max_count = static_cast<size_t>(std::round(max_time / dt));
const hrp::Vector3 init_cog(0, 0, 0.8);

std::vector<hrp::Vector3> cog_list;
std::vector<hrp::Vector3> cogvel_list;
std::vector<hrp::Vector3> cogacc_list;
std::vector<double> time_list;
}

void testPreviewController()
{
    constexpr double preview_time = 1.6;
    constexpr size_t preview_window = static_cast<size_t>(std::round(preview_time / dt));

    cog_list.resize(max_count);
    cogvel_list.resize(max_count);
    cogacc_list.resize(max_count);
    time_list.resize(max_count);

    std::deque<hrp::Vector3> ref_zmp_list;
    ref_zmp_list.emplace_back(0, 0, 0);

    hrp::COGTrajectoryGenerator cog_traj_gen(init_cog);
    cog_traj_gen.initPreviewController(dt, ref_zmp_list.front());

    for (size_t i = 0; i < max_count; ++i) {
        const double cur_time = i * dt;
        time_list.push_back(cur_time);

        while (ref_zmp_list.size() < preview_window) {
            double preview_time = (i + ref_zmp_list.size()) * dt;

            hrp::Vector3 zmp;
            if (preview_time < 1) {
                zmp << 0, 0, 0;
            } else if (preview_time < 3) {
                zmp << 0, 0.02, 0;
            } else if (preview_time < 5) {
                zmp << 0.02, -0.02, 0;
            } else if (preview_time < 7) {
                zmp << 0.04, 0.02, 0;
            } else {
                zmp << 0.04, 0, 0;
            }
            ref_zmp_list.push_back(zmp);
        }

        cog_traj_gen.calcCogFromZMP(ref_zmp_list, dt);

        cog_list[i]    = cog_traj_gen.getCog();
        cogvel_list[i] = cog_traj_gen.getCogVel();
        cogacc_list[i] = cog_traj_gen.getCogAcc();
        time_list[i]   = cur_time;

        ref_zmp_list.pop_front();
    }
}

void testFootGuidedRunning()
{
    const double jump_height = 0.03;
    constexpr double g_acc = 9.80665;
    const double init_cog_z = 0.93;
    const double take_off_z = 0.96;
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    const double flight_time = 2 * take_off_z_vel / g_acc;

    const double support_time = 0.235;
    // constexpr double support_time = 0.5;
    // const double support_time = 0.65;
    // const double support_time = 1.0;
    // const double support_time = 1.2;
    const double step_time = support_time + flight_time;
    const size_t step_count = static_cast<size_t>(std::round(step_time / dt));
    const size_t support_count = static_cast<size_t>(std::round(support_time / dt));

    const double _max_time = step_time * 20;
    const size_t _max_count = static_cast<size_t>(std::round(_max_time / dt));
    cog_list.resize(_max_count);
    cogvel_list.resize(_max_count);
    cogacc_list.resize(_max_count);
    time_list.resize(_max_count);

    std::vector<hrp::Vector3> landing_points;
    std::vector<size_t> landing_counts;
    std::vector<size_t> supporting_counts;
    const hrp::Vector3 _init_cog = hrp::Vector3(0, 0.1, init_cog_z); // Start from left kicking
    // hrp::Vector3 one_step(0.56, -0.2, 0.0);
    hrp::Vector3 one_step(0.525, -0.2, 0.0);
    // hrp::Vector3 one_step(0, -0.2, 0.0);

    const double y_offset = _init_cog[1] > 0 ? -0.05 : 0.05;
    hrp::Vector3 start_zmp_offset = hrp::Vector3(-0.06, y_offset, 0);
    hrp::Vector3 end_zmp_offset = hrp::Vector3(0.16, y_offset, 0);
    // hrp::Vector3 start_zmp_offset = hrp::Vector3(0, y_offset, 0);
    // hrp::Vector3 end_zmp_offset = hrp::Vector3(0, y_offset, 0);
    // hrp::Vector3 start_zmp_offset = hrp::Vector3(0, 0, 0);
    // hrp::Vector3 end_zmp_offset = hrp::Vector3(0, 0, 0);
    // hrp::Vector3 target_cp_offset(0.008, 0, 0);
    hrp::Vector3 target_cp_offset(0, 0, 0);
    // hrp::Vector3 target_cp_offset = hrp::Vector3::Zero();

    // const double omega = std::sqrt(g_acc / _init_cog[2]);
    // target_cp_offset[0] = (one_step[0] + start_zmp_offset[0]) + (one_step[0] + start_zmp_offset[0] - end_zmp_offset[0]) / (omega * flight_time) - one_step[0];
    // std::cerr << "cp offset: "<< target_cp_offset[0] << std::endl;

    landing_points.emplace_back(_init_cog[0], _init_cog[1], 0);
    landing_counts.push_back(500);
    supporting_counts.push_back(support_count);

    hrp::COGTrajectoryGenerator cog_traj_gen(_init_cog);
    for (double count = step_count; count < _max_count * 2; count += step_count) {
        landing_points.push_back(landing_points.back());
        landing_points.back() += one_step;
        landing_counts.push_back(landing_counts.back());
        landing_counts.back() += step_count;
        supporting_counts.push_back(support_count);
        one_step[1] *= -1;

        std::cerr << "landing point: " << landing_points.back().transpose() << std::endl;
        std::cerr << "landing time:  " << landing_counts.back() * dt << std::endl;
    }

    size_t cur_idx = 0;
    hrp::Vector3 prev_cp = _init_cog;
    for (size_t i = 0; i < _max_count; ++i) {
        const double cur_time = i * dt;
        // if (i == 0) start_zmp_offset[0] = 0; // TODO: 最初のoffsetを0にしたい
        // else start_zmp_offset;

        if (i == landing_counts[cur_idx + 1]) {
            ++cur_idx;
            start_zmp_offset = cog_traj_gen.getCog() - landing_points[cur_idx];
            std::cerr << "offset: " << start_zmp_offset.transpose() << std::endl;
            // start_zmp_offset[1] *= -1;
            end_zmp_offset[1] *= -1;
            // _init_cog = cog_list[i - 1];
            std::cerr << "count: " << i << std::endl;
            // std::cerr << "init cog: " << _init_cog.transpose() << std::endl;
            std::cerr << "cur landing: " << landing_points[cur_idx].transpose() << std::endl;
        }

        const hrp::Vector3 s_offset = cur_idx == 0 ? hrp::Vector3::Zero() : start_zmp_offset;

        const hrp::Vector3 ref_zmp = cog_traj_gen.calcCogForRun(landing_points[cur_idx],
                                                                landing_points[cur_idx + 1],
                                                                // start_zmp_offset,
                                                                s_offset,
                                                                end_zmp_offset,
                                                                target_cp_offset,
                                                                take_off_z,
                                                                jump_height,
                                                                landing_counts[cur_idx],
                                                                supporting_counts[cur_idx],
                                                                landing_counts[cur_idx + 1],
                                                                i,
                                                                dt,
                                                                hrp::COGTrajectoryGenerator::CUBIC);

        cog_list[i]    = cog_traj_gen.getCog();
        // cogvel_list[i] = cog_traj_gen.getCogVel();
        cogvel_list[i] = cog_traj_gen.calcCP();
        // cogacc_list[i] = cog_traj_gen.getCogAcc();
        cogacc_list[i] = ref_zmp;
        time_list[i]   = cur_time;

        if (i - landing_counts[cur_idx] == supporting_counts[cur_idx] - 1) {
            const hrp::Vector3 cp = cog_traj_gen.calcCP();
            std::cerr << "cp + dcp*T = " << (cp + (cp - prev_cp) / dt * flight_time).transpose() << std::endl;
        }
        prev_cp = cog_traj_gen.calcCP();
    }
}

void testExtendedMatrixRunning()
{
    const double jump_height = 0.03;
    constexpr double g_acc = 9.80665;
    const double init_cog_z = 0.93;
    const double take_off_z = 0.96;
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    const double flight_time = 2 * take_off_z_vel / g_acc;

    const double support_time = 0.235;
    const double step_time = support_time + flight_time;
    const size_t step_count = static_cast<size_t>(std::round(step_time / dt));
    const size_t support_count = static_cast<size_t>(std::round(support_time / dt));

    const double _max_time = step_time * 60;
    const size_t _max_count = static_cast<size_t>(std::round(_max_time / dt));
    cog_list.resize(_max_count);
    cogvel_list.resize(_max_count);
    cogacc_list.resize(_max_count);
    time_list.resize(_max_count);

    std::vector<hrp::Vector3> landing_points;
    std::vector<size_t> landing_counts;
    std::vector<size_t> supporting_counts;
    const hrp::Vector3 _init_cog = hrp::Vector3(0, 0.1, init_cog_z); // Start from left kicking
    hrp::Vector3 one_step(0.525, -0.2, 0.0);

    const double y_offset = _init_cog[1] > 0 ? -0.03 : 0.03;
    hrp::Vector3 target_cp_offset(0.0, 0, 0);

    landing_points.emplace_back(_init_cog[0], _init_cog[1], 0);
    landing_counts.push_back(500);
    supporting_counts.push_back(support_count);

    for (double count = step_count; count < _max_count * 2; count += step_count) {
        landing_points.push_back(landing_points.back());
        landing_points.back() += one_step;
        landing_counts.push_back(landing_counts.back());
        landing_counts.back() += step_count;
        supporting_counts.push_back(support_count);
        one_step[1] *= -1;

        std::cerr << "landing point: " << landing_points.back().transpose() << std::endl;
        std::cerr << "landing time:  " << landing_counts.back() * dt << std::endl;
    }

    hrp::COGTrajectoryGenerator cog_gen(_init_cog);
    size_t cur_idx = 0;
    hrp::Vector3 ref_zmp = landing_points[0];
    for (size_t count = 0; count < _max_count; ++count) {
        const double cur_time = count * dt;

        // if (count == landing_counts[cur_idx + 1]) {
        //     ++cur_idx;
        //     ref_zmp = landing_points[cur_idx];
        //     ref_zmp[1] += (ref_zmp[1] < 0) ? y_offset : -y_offset;
        //     hrp::Vector3 next_ref_zmp = landing_points[cur_idx + 1];
        //     next_ref_zmp[1] += (next_ref_zmp[1] < 0) ? y_offset : -y_offset;

        //     hrp::Vector3 target_cp = landing_points[cur_idx + 1];
        //     target_cp += target_cp_offset;

        //     // Memo: 0.03608 [ms] 程度
        //     cog_gen.calcCogListForRun(target_cp, ref_zmp, next_ref_zmp, landing_counts[cur_idx + 1] - landing_counts[cur_idx], count,
        //                               jump_height, take_off_z, dt);
        // }

        if (count == landing_counts[cur_idx + 1]) {
            ++cur_idx;
            ref_zmp = landing_points[cur_idx];
            ref_zmp[1] += (ref_zmp[1] < 0) ? y_offset : -y_offset;
            hrp::Vector3 next_ref_zmp = landing_points[cur_idx + 1];
            next_ref_zmp[1] += (next_ref_zmp[1] < 0) ? y_offset : -y_offset;

            hrp::Vector3 last_ref_zmp = landing_points[cur_idx + 2];
            last_ref_zmp[1] += (last_ref_zmp[1] < 0) ? y_offset : -y_offset;

            hrp::Vector3 target_cp = landing_points[cur_idx + 2];
            target_cp += target_cp_offset;

            // std::chrono::system_clock::time_point  start, end;
            // start = std::chrono::system_clock::now();
            // constexpr size_t LOOP = 1000;

            // for (size_t i = 0; i < LOOP; ++i) {

            // Memo: 0.15 [ms] 程度
            cog_gen.calcCogListForRun2Step(target_cp, ref_zmp, next_ref_zmp, last_ref_zmp,
                                           landing_counts[cur_idx + 1] - landing_counts[cur_idx],
                                           landing_counts[cur_idx + 2] - landing_counts[cur_idx + 1],
                                           count,
                                           jump_height, jump_height, take_off_z, take_off_z, dt);
            // }
            // end = std::chrono::system_clock::now();  // 計測終了時間
            // double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            // std::cerr << "time: " << elapsed / LOOP << " [ms]" << std::endl;
        }

        cog_gen.getCogFromCogList(count, dt);

        cog_list[count]    = cog_gen.getCog();
        // cogvel_list[count] = cog_gen.getCogVel();
        // cogvel_list[count] = cog_gen.calcCP();
        cogvel_list[count] = cog_gen.calcPointMassZMP() - ref_zmp;
        cogacc_list[count] = cog_gen.getCogAcc();
        time_list[count]   = cur_time;
    }
}

int main(int argc, char **argv)
{
    std::vector<std::string> arg_strs;
    for (int i = 1; i < argc; ++i) {
        arg_strs.push_back(std::string(argv[i]));
    }

    bool use_gnuplot = true;

    if (argc > 2) {
        if (std::string(argv[1]) == "--use-gnuplot") {
            use_gnuplot = (std::string(argv[2]) == "true");
        }
    }

    // testPreviewController(cog_list, time_list);
    // testFootGuidedRunning();
    testExtendedMatrixRunning();

    const std::string fname("/tmp/testCOGTrajectoryGenerator.dat");
    std::ofstream ofs(fname);

    for (size_t i = 0; i < cog_list.size(); ++i) {
        ofs << time_list[i] << " "
            << cog_list[i][0] << " " << cog_list[i][1] << " " << cog_list[i][2] << " "
            << cogvel_list[i][0] << " " << cogvel_list[i][1] << " " << cogvel_list[i][2] << " "
            << cogacc_list[i][0] << " " << cogacc_list[i][1] << " " << cogacc_list[i][2] << " "
            << std::endl;
    }

    ofs.close();

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 3, 1\n");
        fprintf(gp, "set xlabel \"time [s]\"\n");
        fprintf(gp, "set ylabel \"COG [m]\"\n");
        fprintf(gp, "set title \"COG X\"\n");
        fprintf(gp, "plot \"%s\" using 1:2 with lines title \"Cog X\"\n", fname.c_str());
        fprintf(gp, "set title \"COG Y\"\n");
        fprintf(gp, "plot \"%s\" using 1:3 with lines title \"Cog Y\"\n", fname.c_str());
        fprintf(gp, "set title \"COG Z\"\n");
        fprintf(gp, "plot \"%s\" using 1:4 with lines title \"Cog Z\"\n", fname.c_str());
        fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }

    return 0;
}
