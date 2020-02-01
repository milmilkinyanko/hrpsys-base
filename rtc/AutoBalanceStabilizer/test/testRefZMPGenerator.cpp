// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testRefZMPGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <iostream>
#include <cstdio>
#include <fstream>
#include "../RefZMPGenerator.h"

namespace
{

std::vector<hrp::Vector3> rleg_contact_points;
std::vector<hrp::Vector3> lleg_contact_points;

void forwardFootstep(std::vector<hrp::ConstraintsWithCount>& constraints_list,
                     const double stride, const size_t step_limb,
                     const size_t start_count, const size_t step_count)
{
    constraints_list.reserve(constraints_list.size() + 2);

    const std::vector<hrp::LinkConstraint>& cur_constraints = constraints_list.back().constraints;

    {
        // Swinging phase
        hrp::ConstraintsWithCount const_with_count;
        const_with_count.start_count = start_count;
        const_with_count.constraints = cur_constraints;
        const_with_count.constraints[step_limb].setConstraintType(hrp::LinkConstraint::FLOAT);

        constraints_list.push_back(const_with_count);
    }

    {
        // Double support phase
        hrp::ConstraintsWithCount const_with_count;
        const_with_count.start_count = start_count + step_count;
        const_with_count.constraints = cur_constraints;
        const_with_count.constraints[step_limb].targetPos() += hrp::Vector3(stride, 0, 0);

        constraints_list.push_back(const_with_count);
    }
}

void initConstraints(std::vector<hrp::ConstraintsWithCount>& constraints_list, const size_t start_count)
{
    hrp::ConstraintsWithCount const_with_count;
    const_with_count.start_count = start_count;

    {
        hrp::LinkConstraint rleg_constraint(0);
        for (const auto& point : rleg_contact_points) rleg_constraint.addLinkContactPoint(point);
        rleg_constraint.calcLinkLocalPos();
        rleg_constraint.targetPos() = hrp::Vector3(0, -0.1, 0);
        const_with_count.constraints.push_back(rleg_constraint);
    }

    {
        hrp::LinkConstraint lleg_constraint(1);
        for (const auto& point : lleg_contact_points) lleg_constraint.addLinkContactPoint(point);
        lleg_constraint.calcLinkLocalPos();
        lleg_constraint.targetPos() = hrp::Vector3(0, 0.1, 0);
        const_with_count.constraints.push_back(lleg_constraint);
    }

    constraints_list.push_back(const_with_count);
}

}

int main(int argc, char **argv)
{
    constexpr double dt = 0.002;

    bool use_gnuplot = true;
    if (argc > 2) {
        if (std::string(argv[1]) == "--use-gnuplot") {
            use_gnuplot = (std::string(argv[2]) == "true");
        }
    }

    rleg_contact_points.emplace_back(0.15, 0.05, 0);
    rleg_contact_points.emplace_back(0.15, -0.05, 0);
    rleg_contact_points.emplace_back(-0.15, -0.05, 0);
    rleg_contact_points.emplace_back(-0.15, 0.05, 0);

    lleg_contact_points.emplace_back(0.15, 0.05, 0);
    lleg_contact_points.emplace_back(0.15, -0.05, 0);
    lleg_contact_points.emplace_back(-0.15, -0.05, 0);
    lleg_contact_points.emplace_back(-0.15, 0.05, 0);

    std::vector<hrp::ConstraintsWithCount> constraints_list;

    size_t start_count = 0;
    constexpr size_t SUPPORT_COUNT = static_cast<size_t>(1.0 / dt);
    constexpr size_t STEP_COUNT = static_cast<size_t>(1.0 / dt);

    initConstraints(constraints_list, start_count);

    start_count += SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.4, 0, start_count, STEP_COUNT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.8, 1, start_count, STEP_COUNT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.8, 0, start_count, STEP_COUNT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.4, 1, start_count, STEP_COUNT);

    const size_t FINISH_COUNT = start_count + STEP_COUNT + SUPPORT_COUNT + static_cast<size_t>(3.0 / dt);
    hrp::RefZMPGenerator zmp_generator(dt, FINISH_COUNT, constraints_list[0]);
    zmp_generator.setRefZMPList(constraints_list, 0);

    const std::string fname("/tmp/testRefZMP.dat");
    {
        std::ofstream ofs(fname);

        size_t count = 0;
        for (const auto& ref_zmp : zmp_generator.getRefZMPList()) {
            ofs << count * dt << " " << ref_zmp(0) << " " << ref_zmp(1) << " " << ref_zmp(2) << std::endl;
            ++count;
        }
        ofs.close();
    }

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 2, 1\n");
        fprintf(gp, "set title \"ZMP\"\n");
        fprintf(gp, "set xlabel \"time [s]\"\n");
        fprintf(gp, "plot \"%s\" using 1:2 with lines title \"X\"\n", fname.c_str());
        fprintf(gp, "plot \"%s\" using 1:3 with lines title \"Y\"\n", fname.c_str());
        // fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }

    return 0;
}
