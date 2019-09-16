// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LimbTrajectoryGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <iostream>
#include "LimbTrajectoryGenerator.h"
#include "Utility.h"

namespace {

hrp::Vector3 calcCycloidMidPoint(const double ratio, const hrp::Vector3& start,
                              const hrp::Vector3& goal, const double height,
                              const double default_top_ratio = 0.5)
{
    hrp::Vector3 u ( goal - start );
    const hrp::Vector3 uz (0, 0, ratio * u(2));
    u(2) = 0.0;

    const double pth = 2 * M_PI * ratio;
    const double norm_u = u.norm();

    if (!hrp::eps_eq(norm_u, 0.0, 1e-3 * 0.01)) u = u.normalized();

    /* check ratio vs 0.5 for default_top_ratio blending */
    const hrp::Vector3 cycloid_point( ((0.5 > ratio) ? ( 2 * default_top_ratio * norm_u ) : ( 2 * (1 - default_top_ratio) * norm_u )) *
                                      ( pth - sin(pth) ) / (2 * M_PI) -
                                      ((0.5 > ratio) ? 0.0 : (norm_u * (1 - 2 * default_top_ratio)) ), // local x
                                      0, // local y
                                      ( 0.5 * height * ( 1 - cos(pth) )) ); // local z

    const hrp::Vector3 v(hrp::Vector3(0,0,1).cross(u));
    hrp::Matrix33 dvm;
    dvm << u(0), v(0), 0,
        u(1), v(1), 0,
        u(2), v(2), 1;

    return dvm * cycloid_point + start + uz;
}


std::tuple<hrp::Vector3, hrp::Vector3, hrp::Vector3>
hoffArbibInterpolation(const double remain_time, const double dt,
                       const hrp::Vector3& _pos, const hrp::Vector3& _vel,
                       const hrp::Vector3& _acc,
                       const hrp::Vector3& goal_pos,
                       const hrp::Vector3& goal_vel = hrp::Vector3::Zero(),
                       const hrp::Vector3& goal_acc = hrp::Vector3::Zero())
{
    const hrp::Vector3 jerk =
        (-9.0 / remain_time) * (_acc - goal_acc / 3.0) +
        (-36.0 / (remain_time * remain_time)) * (goal_vel * 2.0 / 3.0 + _vel) +
        (60.0 / (remain_time * remain_time * remain_time)) * (goal_pos - _pos);

    const hrp::Vector3 acc = _acc + dt * jerk;
    const hrp::Vector3 vel = _vel + dt * acc;
    const hrp::Vector3 pos = _pos + dt * vel;
    return std::forward_as_tuple(pos, vel, acc);
}

}

namespace hrp {

void LimbTrajectoryGenerator::calcTrajectory(const size_t count, const double dt)
{
    if (via_points.empty()) return;

    if (traj_type > DELAY_HOFFARBIB) calcDelayHoffArbibTrajectory(count, dt);
}

// Calculate Hoff & Arbib trajectory following preceding point by delay_time_offset [s]
void LimbTrajectoryGenerator::calcDelayHoffArbibTrajectory(const size_t count, const double dt)
{
    size_t idx = 0;
    const double preceding_count = count + static_cast<size_t>(delay_time_offset / dt);

    for (size_t i = 0; i < via_points.size() && via_points[i].count < preceding_count; ++i) idx = i;

    hrp::Vector3 target;
    if (idx == via_points.size() - 1)      target = via_points.back().point;
    else if (preceding_count <= via_points[0].count) target = via_points[0].point;
    else {
        const double ratio = static_cast<double>(preceding_count - via_points[idx].count) / (via_points[idx + 1].count - via_points[idx].count);
        target = calcInteriorPoint(via_points[idx].point, via_points[idx + 1].point, ratio);
    }

    std::tie(pos, vel, acc) = hoffArbibInterpolation(delay_time_offset, dt, pos, vel, acc, target);
}

void LimbTrajectoryGenerator::calcViaPoints(const TrajectoryType _traj_type,
                                            const hrp::Vector3& start, const hrp::Vector3& goal,
                                            const size_t start_count, const size_t goal_count,
                                            const double height)
{
    switch (_traj_type) {
      case CYCLOIDDELAY:
          calcCycloidDelayViaPoints(start, goal, start_count, goal_count, height);
          break;
      case RECTANGLE:
          calcRectangleViaPoints(start, goal, start_count, goal_count, height);
          break;
      default:
          std::cerr << "[LimbTrajectoryGenerator] Please select a appropreate trajectory type" << std::endl;
          break;
    }
}

// TODO: この辺はcountを渡すのではなくindexを渡すのでも良い
void LimbTrajectoryGenerator::calcViaPoints(const TrajectoryType _traj_type,
                                            const std::vector<ConstraintsWithCount>& constraints_list,
                                            const int link_id, const size_t count, const double height)
{
    // TODO: use next_turning_count
    const size_t cur_idx = getConstraintIndexFromCount(constraints_list, count);
    size_t goal_idx = cur_idx;

    int goal_constraint_idx;
    while (goal_idx < constraints_list.size()) {
        // TODO: constraintのindexを渡してしまうのも手．その場合前後でconstraints_listの数が変わるとまずい
        goal_constraint_idx = constraints_list[goal_idx].getConstraintIndexFromLinkId(link_id);
        if (goal_constraint_idx == -1) break;
        if (constraints_list[goal_idx].constraints[goal_constraint_idx].getConstraintType() == LinkConstraint::FLOAT) ++goal_idx;
        else break;
    }

    if (goal_idx == cur_idx) {
        std::cerr << "[LimbTrajectoryGenerator] The constraint " << link_id << " is not FLOAT" << std::endl;
        return;
    }

    if (goal_constraint_idx == -1 || goal_idx == constraints_list.size()) {
        std::cerr << "[LimbTrajectoryGenerator] Can't find landing position" << std::endl;
        return;
    }

    traj_type = _traj_type;
    int start_constraint_idx = constraints_list[cur_idx].getConstraintIndexFromLinkId(link_id);
    if (start_constraint_idx == -1) {
        std::cerr << "[LimbTrajectoryGenerator] Can't find start constraint" << std::endl;
        return;
    }

    calcViaPoints(traj_type,
                  constraints_list[cur_idx].constraints[start_constraint_idx].targetPos(),
                  constraints_list[goal_idx].constraints[goal_constraint_idx].targetPos(),
                  constraints_list[cur_idx].start_count, constraints_list[goal_idx].start_count,
                  height);
}

void LimbTrajectoryGenerator::calcCycloidDelayViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                                        const size_t start_count, const size_t goal_count,
                                                        const double height)
{
    via_points.resize(7);

    hrp::Vector3 via_goal(goal);
    constexpr double ratio = 0.4;
    via_goal(2) += ratio * height;
    const double tmpheight = height + (goal(2) - via_goal(2)) / 2.0; // TODO: rename

    const size_t diff_count = goal_count - start_count;

    via_points[0].point = start;
    via_points[0].count = start_count;

    via_points[1].point = calcCycloidMidPoint(0.2, start, via_goal, tmpheight);
    via_points[1].count = start_count + static_cast<size_t>(0.2 * diff_count);

    via_points[2].point = calcCycloidMidPoint(0.4, start, via_goal, tmpheight);
    via_points[2].count = start_count + static_cast<size_t>(0.4 * diff_count);

    via_points[3].point = calcCycloidMidPoint(0.6, start, via_goal, tmpheight);
    via_points[3].count = start_count + static_cast<size_t>(0.6 * diff_count);

    via_points[4].point = calcCycloidMidPoint(0.8, start, via_goal, tmpheight);
    via_points[4].count = start_count + static_cast<size_t>(0.8 * diff_count);

    via_points[5].point = via_goal;
    {
        double sum_distance = 0;
        const double distance_to_via_goal = (via_goal - via_points[4].point).norm();
        sum_distance += distance_to_via_goal;
        sum_distance += ratio * height;

        via_points[5].count = start_count + static_cast<size_t>(0.8 * diff_count + distance_to_via_goal / sum_distance);
    }

    via_points[6].point = goal;
    via_points[6].count = goal_count;
}

void LimbTrajectoryGenerator::calcRectangleViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                                     const size_t start_count, const size_t goal_count,
                                                     const double height)
{
    via_points.resize(4);
    const double max_height = std::max(start(2), goal(2)) + height;
    const double first_distance = max_height - start(2);
    const double second_distance = (start.head<2>() - goal.head<2>()).norm();
    const double third_distance = max_height - goal(2);
    const double sum_distance = first_distance + second_distance + third_distance;
    const size_t diff_count = goal_count - start_count;

    via_points[0].point = start;
    via_points[0].count = start_count;

    via_points[1].point = hrp::Vector3(start(0), start(1), max_height);
    via_points[1].count = start_count + static_cast<size_t>(first_distance / sum_distance * diff_count);

    via_points[2].point = hrp::Vector3(goal(0), goal(1), max_height);
    via_points[2].count = start_count + static_cast<size_t>(second_distance / sum_distance * diff_count);

    via_points[3].point = goal;
    via_points[3].count = goal_count;
}

}
