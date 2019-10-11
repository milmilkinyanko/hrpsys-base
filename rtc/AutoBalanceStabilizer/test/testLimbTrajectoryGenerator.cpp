// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testLimbTrajectoryGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <cstdio>
#include <iostream>
#include <fstream>
#include "../LinkConstraint.h"

namespace
{

void forwardFootstep(std::vector<hrp::ConstraintsWithCount>& constraints_list,
                     const double stride, const size_t step_limb,
                     const size_t start_count, const size_t step_count,
                     const double step_height)
{
    constraints_list.reserve(constraints_list.size() + 2);

    const std::vector<hrp::LinkConstraint>& cur_constraints = constraints_list.back().constraints;

    {
        // Swinging phase
        hrp::ConstraintsWithCount swing_constraints;
        swing_constraints.start_count = start_count;
        swing_constraints.constraints = cur_constraints;
        swing_constraints.clearLimbViaPoints();
        swing_constraints.constraints[step_limb].setConstraintType(hrp::LinkConstraint::FLOAT);

        constraints_list.push_back(swing_constraints);
    }

    {
        // Double support phase
        hrp::ConstraintsWithCount landing_constraint;
        landing_constraint.start_count = start_count + step_count;
        landing_constraint.constraints = cur_constraints;
        landing_constraint.clearLimbViaPoints();
        landing_constraint.constraints[step_limb].targetPos() += hrp::Vector3(stride, 0, 0);

        constraints_list.back().constraints[step_limb].calcLimbViaPoints(hrp::LimbTrajectoryGenerator::CYCLOIDDELAY,
                                                                         landing_constraint.constraints[step_limb].targetCoord(),
                                                                         start_count, landing_constraint.start_count,
                                                                         step_height);

        constraints_list.push_back(landing_constraint);
    }
}

void initConstraints(std::vector<hrp::ConstraintsWithCount>& constraints_list, const size_t start_count)
{
    hrp::ConstraintsWithCount const_with_count;
    const_with_count.start_count = start_count;

    {
        hrp::LinkConstraint rleg_constraint(0);
        rleg_constraint.targetPos() = hrp::Vector3(0, -0.1, 0);
        const_with_count.constraints.push_back(rleg_constraint);
    }

    {
        hrp::LinkConstraint lleg_constraint(1);
        lleg_constraint.targetPos() = hrp::Vector3(0, 0.1, 0);
        const_with_count.constraints.push_back(lleg_constraint);
    }

    constraints_list.push_back(const_with_count);
}

}

int main(int argc, char **argv)
{
    // same as testRefZMP
    constexpr double dt = 0.002;

    bool use_gnuplot = true;
    if (argc > 2) {
        if (std::string(argv[1]) == "--use-gnuplot") {
            use_gnuplot = (std::string(argv[2]) == "true");
        }
    }

    std::vector<hrp::ConstraintsWithCount> constraints_list;

    size_t start_count = 0;
    constexpr size_t SUPPORT_COUNT = static_cast<size_t>(1.0 / dt);
    constexpr size_t STEP_COUNT = static_cast<size_t>(1.0 / dt);

    initConstraints(constraints_list, start_count);
    constexpr double FOOTSTEP_HEIGHT = 0.15;

    start_count += SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.4, 0, start_count, STEP_COUNT, FOOTSTEP_HEIGHT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.8, 1, start_count, STEP_COUNT, FOOTSTEP_HEIGHT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.8, 0, start_count, STEP_COUNT, FOOTSTEP_HEIGHT);

    start_count += STEP_COUNT + SUPPORT_COUNT;
    forwardFootstep(constraints_list, 0.4, 1, start_count, STEP_COUNT, FOOTSTEP_HEIGHT);

    size_t next_turning_count = 0;
    int index = -1;
    const size_t FINISH = start_count + STEP_COUNT + SUPPORT_COUNT;

    const std::string fname("/tmp/testLimbTrajectory.dat");
    std::ofstream ofs(fname);

    for (size_t count = 0; count < FINISH; ++count) {
        if (count == next_turning_count) {
            ++index;
            if (index + 1 < constraints_list.size()) next_turning_count = constraints_list[index + 1].start_count;
            if (index > 0) constraints_list[index].copyLimbState(constraints_list[index - 1]);
        }

        constraints_list[index].calcLimbTrajectory(count, dt);
        for (const auto& constraint : constraints_list[index].constraints) {
            const hrp::Vector3& pos = constraint.targetPos();
            ofs << pos[0] << " " << pos[1] << " " << pos[2] << " ";
        }
        ofs << std::endl;
    }

    ofs.close();

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 2, 1\n");
        fprintf(gp, "set title \"Pos\"\n");
        fprintf(gp, "plot \"%s\" using 1:3 with lines title \"Right\"\n", fname.c_str());
        fprintf(gp, "plot \"%s\" using 4:6 with lines title \"Left\"\n", fname.c_str());
        fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }


    return 0;
}
