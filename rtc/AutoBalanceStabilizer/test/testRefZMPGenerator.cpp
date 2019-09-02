// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testRefZMPGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <cstdio>
#include <fstream>
#include "../RefZMPGenerator.h"

namespace
{

std::vector<hrp::Vector3> rleg_contact_points;
std::vector<hrp::Vector3> lleg_contact_points;

void forwardFootstep(std::vector<hrp::Vector3>& contact_points, const double stride)
{
    for (auto& point : contact_points) point += hrp::Vector3(stride, 0, 0);
}

void addConstraints(std::vector<hrp::ConstraintsWithCount>& constraints_list, const size_t start_count,
                    const int leg_mask = 0b11, const double rleg_weight = 1.0, const double lleg_weight = 1.0)
{
    if (!(leg_mask & 0b11)) return;

    hrp::ConstraintsWithCount constraint_count;
    constraint_count.start_count = start_count;

    if (leg_mask & 0b10) {
        hrp::LinkConstraint rleg_constraint(0);
        for (const auto& point : rleg_contact_points) rleg_constraint.addLinkContactPoint(point);
        rleg_constraint.calcLinkRepresentativePoint();
        rleg_constraint.setWeight(rleg_weight);
        constraint_count.constraints.push_back(rleg_constraint);
    }

    if (leg_mask & 0b01) {
        hrp::LinkConstraint lleg_constraint(1);
        for (const auto& point : lleg_contact_points) lleg_constraint.addLinkContactPoint(point);
        lleg_constraint.calcLinkRepresentativePoint();
        lleg_constraint.setWeight(lleg_weight);
        constraint_count.constraints.push_back(lleg_constraint);
    }

    constraints_list.push_back(constraint_count);
}

}

int main(int argc, char **argv)
{
    constexpr double dt = 0.002;
    constexpr size_t LIST_SIZE = 5.0 / dt;

    bool use_gnuplot = true;
    if (argc > 2) {
        if (std::string(argv[1]) == "--use-gnuplot") {
            use_gnuplot = (std::string(argv[2]) == "true");
        }
    }

    std::vector<hrp::ConstraintsWithCount> constraints_list;
    hrp::RefZMPGenerator zmp_generator;

    rleg_contact_points.emplace_back(0.15, 0.05, 0);
    rleg_contact_points.emplace_back(0.15, 0.15, 0);
    rleg_contact_points.emplace_back(-0.15, 0.15, 0);
    rleg_contact_points.emplace_back(-0.15, 0.05, 0);

    lleg_contact_points.emplace_back(0.15, -0.05, 0);
    lleg_contact_points.emplace_back(0.15, -0.15, 0);
    lleg_contact_points.emplace_back(-0.15, -0.15, 0);
    lleg_contact_points.emplace_back(-0.15, -0.05, 0);

    addConstraints(constraints_list, 0);
    addConstraints(constraints_list, static_cast<size_t>(0.5 / dt), 0b01);

    // TODO: 動かし方が違う．
    //       contact_point はローカル座標の点の話で，動かすべきはtarget_coord
    forwardFootstep(rleg_contact_points, 0.4);
    addConstraints(constraints_list, static_cast<size_t>(1.0 / dt), 0b11, 1.0, 0.0);
    addConstraints(constraints_list, static_cast<size_t>(1.5 / dt), 0b10);

    forwardFootstep(lleg_contact_points, 0.8);
    addConstraints(constraints_list, static_cast<size_t>(2.0 / dt), 0b11, 0.0, 1.0);
    addConstraints(constraints_list, static_cast<size_t>(2.5 / dt), 0b01);

    forwardFootstep(rleg_contact_points, 0.8);
    addConstraints(constraints_list, static_cast<size_t>(3.0 / dt), 0b11, 1.0, 0.0);
    addConstraints(constraints_list, static_cast<size_t>(3.5 / dt), 0b10);

    forwardFootstep(lleg_contact_points, 0.4);
    addConstraints(constraints_list, static_cast<size_t>(4.0 / dt));

    zmp_generator.setRefZMPListUsingConstraintList(constraints_list, LIST_SIZE);


    const std::string fname("/tmp/testRefZMP.dat");
    std::ofstream ofs(fname);

    for (const auto& ref_zmp : zmp_generator.getRefZMPList()) {
        ofs << ref_zmp(0) << " " << ref_zmp(1) << " " << ref_zmp(2) << std::endl;
    }
    ofs.close();

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 1, 2\n");
        fprintf(gp, "set title \"ZMP\"\n");
        fprintf(gp, "plot \"%s\" using 1 with lines title \"X\"\n", fname.c_str());
        fprintf(gp, "plot \"%s\" using 2 with lines title \"Y\"\n", fname.c_str());
        fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }

    return 0;
}
