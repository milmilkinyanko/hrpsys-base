// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testCOGTrajectoryGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <cstdio>
#include <fstream>
#include "../COGTrajectoryGenerator.h"

namespace
{
constexpr double dt = 0.002;
constexpr double max_time = 8.0;
constexpr size_t max_count = static_cast<size_t>(std::round(max_time / dt));
const hrp::Vector3 init_cog(0, 0, 0.8);
}

void testPreviewController(std::vector<hrp::Vector3>& cog_list, std::vector<double>& time_list)
{
    constexpr double preview_time = 1.6;
    constexpr size_t preview_window = static_cast<size_t>(std::round(preview_time / dt));

    cog_list.resize(max_count);
    time_list.resize(max_count);

    std::deque<hrp::Vector3> ref_zmp_list;
    ref_zmp_list.emplace_back(0, 0, 0);


    hrp::COGTrajectoryGenerator cog_traj_gen(init_cog);
    cog_traj_gen.initPreviewController(dt, ref_zmp_list.front());

    for (size_t i = 0; i < max_count; ++i) {
        double cur_time = i * dt;
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

        cog_traj_gen.calcCogFromZMP(ref_zmp_list);

        cog_list[i] = cog_traj_gen.getCog();
        time_list[i] = cur_time;

        ref_zmp_list.pop_front();
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

    std::vector<hrp::Vector3> cog_list;
    std::vector<double> time_list;

    testPreviewController(cog_list, time_list);

    const std::string fname("/tmp/testCOGTrajectoryGenerator.dat");
    std::ofstream ofs(fname);

    for (size_t i = 0; i < cog_list.size(); ++i) {
        ofs << time_list[i] << " " << cog_list[i][0] << " " << cog_list[i][1] << " " << cog_list[i][2] << " " << std::endl;
    }

    ofs.close();

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 2, 1\n");
        fprintf(gp, "set title \"Pos\"\n");
        fprintf(gp, "plot \"%s\" using 1:2 with lines title \"Cog X\"\n", fname.c_str());
        fprintf(gp, "plot \"%s\" using 1:3 with lines title \"Cog Y\"\n", fname.c_str());
        fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }

    return 0;
}
