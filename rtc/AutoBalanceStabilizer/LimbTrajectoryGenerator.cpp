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

std::tuple<double, double, double>
linearInterpolation(const double remain_time, const double dt,
                    const double _pos, const double goal_pos)
{
    const double acc = 0;
    const double vel = (goal_pos - _pos) / remain_time;
    const double pos = _pos + dt * vel;
    return std::forward_as_tuple(pos, vel, acc);
}

std::tuple<hrp::Vector3, hrp::Vector3, hrp::Vector3>
linearInterpolation(const double remain_time, const double dt,
                    const hrp::Vector3& _pos, const hrp::Vector3& goal_pos)
{
    const hrp::Vector3 acc = hrp::Vector3::Zero();
    const hrp::Vector3 vel = (goal_pos - _pos) / remain_time;
    const hrp::Vector3 pos = _pos + dt * vel;
    return std::forward_as_tuple(pos, vel, acc);
}

std::tuple<double, double, double>
hoffArbibInterpolation(const double remain_time, const double dt,
                       const double _pos, const double _vel,
                       const double _acc,
                       const double goal_pos,
                       const double goal_vel = 0,
                       const double goal_acc = 0)
{
    const double jerk =
        (-9.0 / remain_time) * (_acc - goal_acc / 3.0) +
        (-36.0 / (remain_time * remain_time)) * (goal_vel * 2.0 / 3.0 + _vel) +
        (60.0 / (remain_time * remain_time * remain_time)) * (goal_pos - _pos);

    const double acc = _acc + dt * jerk;
    const double vel = _vel + dt * acc;
    const double pos = _pos + dt * vel;
    return std::forward_as_tuple(pos, vel, acc);
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
        (-9.0  / remain_time) * (_acc - goal_acc / 3.0) +
        (-36.0 / (remain_time * remain_time)) * (goal_vel * 2.0 / 3.0 + _vel) +
        (60.0  / (remain_time * remain_time * remain_time)) * (goal_pos - _pos);

    const hrp::Vector3 acc = _acc + dt * jerk;
    const hrp::Vector3 vel = _vel + dt * acc;
    const hrp::Vector3 pos = _pos + dt * vel;
    return std::forward_as_tuple(pos, vel, acc);
}

}

namespace hrp {

void LimbTrajectoryGenerator::calcTrajectory(const size_t count, const double dt,
                                             Eigen::Ref<Eigen::Vector3d> pos, Eigen::Ref<Eigen::Vector3d> vel,
                                             Eigen::Ref<Eigen::Vector3d> acc,
                                             Eigen::Ref<Eigen::Matrix3d> rot, double& rot_vel,
                                             double& rot_acc)
{
    if (via_points.empty() || via_points.back().count <= count) return;

    if (traj_type >= DELAY_HOFFARBIB) calcDelayHoffArbibTrajectory(count, dt, pos, vel, acc, rot, rot_vel, rot_acc);
    else if (traj_type == LINEAR) calcLinearTrajectory(count, dt, pos, vel, acc, rot, rot_vel, rot_acc);
}

std::tuple<size_t, double, hrp::Vector3, double> LimbTrajectoryGenerator::calcTarget(const size_t count, const double dt)
{
    size_t idx = 0;
    const double preceding_count = count + static_cast<size_t>(delay_time_offset / dt);
    double remain_time = delay_time_offset;

    for (size_t i = 0; i < via_points.size() && via_points[i].count < preceding_count; ++i) idx = i;

    hrp::Vector3 target;
    double rot_angle;
    if (idx == via_points.size() - 1) {
        target = via_points.back().point;
        rot_angle = via_points.back().diff_rot_angle;
        remain_time = (via_points.back().count - count) * dt;
    } else if (preceding_count <= via_points[0].count) {
        target = via_points[0].point;
        rot_angle = 0;
    } else {
        const double ratio = static_cast<double>(preceding_count - via_points[idx].count) / (via_points[idx + 1].count - via_points[idx].count);
        target = calcInteriorPoint(via_points[idx].point, via_points[idx + 1].point, ratio);

        const double rot_ratio = static_cast<double>(preceding_count - via_points[0].count) / (via_points.back().count - via_points[0].count);
        rot_angle = calcInteriorPoint(0.0, via_points.back().diff_rot_angle, rot_ratio);
    }

    return std::forward_as_tuple(idx, remain_time, target, rot_angle);
}

void LimbTrajectoryGenerator::calcLinearTrajectory(const size_t count, const double dt,
                                                   Eigen::Ref<Eigen::Vector3d> pos, Eigen::Ref<Eigen::Vector3d> vel,
                                                   Eigen::Ref<Eigen::Vector3d> acc,
                                                   Eigen::Ref<Eigen::Matrix3d> rot, double& rot_vel,
                                                   double& rot_acc)
{
    double remain_time;
    hrp::Vector3 target;
    double rot_angle;
    std::tie(std::ignore, remain_time, target, rot_angle) = calcTarget(count, dt);

    std::tie(pos, vel, acc) = linearInterpolation(remain_time, dt, pos, target);
    std::tie(diff_rot.angle(), rot_vel, rot_acc) = linearInterpolation(remain_time, dt, diff_rot.angle(), rot_angle);
    rot = start_rot * diff_rot.toRotationMatrix();

    // std::cerr << "linear pos: " << pos.transpose() << ", target: " << target.transpose() << ", rot angle: " << diff_rot.angle() << std::endl;
}

// Calculate Hoff & Arbib trajectory following preceding point by delay_time_offset [s]
void LimbTrajectoryGenerator::calcDelayHoffArbibTrajectory(const size_t count, const double dt,
                                                           Eigen::Ref<Eigen::Vector3d> pos, Eigen::Ref<Eigen::Vector3d> vel,
                                                           Eigen::Ref<Eigen::Vector3d> acc,
                                                           Eigen::Ref<Eigen::Matrix3d> rot, double& rot_vel,
                                                           double& rot_acc)
{
    double remain_time;
    hrp::Vector3 target;
    double rot_angle;
    std::tie(std::ignore, remain_time, target, rot_angle) = calcTarget(count, dt);

    std::tie(pos, vel, acc) = hoffArbibInterpolation(remain_time, dt, pos, vel, acc, target);
    std::tie(diff_rot.angle(), rot_vel, rot_acc) = hoffArbibInterpolation(remain_time, dt, diff_rot.angle(), rot_vel, rot_acc, rot_angle);
    rot = start_rot * diff_rot.toRotationMatrix();

    std::cerr << "remain_time: " << remain_time << ", target: " << target.transpose() << ", pos: " << pos.transpose() << std::endl;
}

void LimbTrajectoryGenerator::calcViaPoints(const TrajectoryType _traj_type,
                                            const Eigen::Isometry3d& start, const Eigen::Isometry3d& goal,
                                            const size_t start_count, const size_t goal_count,
                                            const double height)
{
    clearViaPoints();
    traj_type = _traj_type;

    switch (_traj_type) {
        // TODO: calcRotationViaPointsを消してLinearを足す？
      case CYCLOIDDELAY:
          calcCycloidDelayViaPoints(start.translation(), goal.translation(), start_count, goal_count, height);
          break;
      case RECTANGLE:
          calcRectangleViaPoints(start.translation(), goal.translation(), start_count, goal_count, height);
          break;
      default:
          std::cerr << "[LimbTrajectoryGenerator] Please select a appropreate trajectory type" << std::endl;
          break;
    }

    if (!via_points.empty()) {
        start_rot = start.linear();
        diff_rot = Eigen::AngleAxisd(start.linear().transpose() * goal.linear());
        via_points.back().diff_rot_angle = diff_rot.angle();
        diff_rot.angle() = 0;
    }

    // Debug
    std::cerr << "via points:" << std::endl;
    for (auto&& point : via_points) {
        std::cerr << point.point.transpose() << std::endl;
    }
    std::cerr << "start_count: " << start_count << ", goal_count: " << goal_count << std::endl;
}

void LimbTrajectoryGenerator::setViaPoints(const TrajectoryType _traj_type,
                                           const Eigen::Isometry3d& start,
                                           const Eigen::Isometry3d& goal,
                                           const std::vector<ViaPoint>& _via_points)
{
    via_points = _via_points;
    traj_type = _traj_type;
    start_rot = start.linear();
    diff_rot = Eigen::AngleAxisd(start.linear().transpose() * goal.linear());
    via_points.back().diff_rot_angle = diff_rot.angle();
    diff_rot.angle() = 0;
}

void LimbTrajectoryGenerator::calcRotationViaPoints(const TrajectoryType _traj_type,
                                                    const Eigen::Isometry3d& start,
                                                    const Eigen::Vector3d& rot_axis,
                                                    const double rot_angle,
                                                    const size_t start_count,
                                                    const size_t goal_count)
{
    std::cerr << "calc limb rot" << std::endl;
    traj_type = _traj_type;
    start_rot = start.linear();
    diff_rot = Eigen::AngleAxisd(0, rot_axis);

    via_points.clear();
    via_points.resize(2);

    via_points[0].point = start.translation();
    via_points[0].count = start_count;

    via_points[1].point = start.translation();
    via_points[1].count = goal_count;
    via_points[1].diff_rot_angle = rot_angle;
    std::cerr << "goal point: " << via_points[1].point.transpose() << std::endl;
    std::cerr << "goal rot angle: " << via_points[1].diff_rot_angle << std::endl;
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

void LimbTrajectoryGenerator::modifyViaPoints(const Eigen::Isometry3d& current_coord,
                                              const Eigen::Isometry3d& new_goal,
                                              const size_t current_count,
                                              const size_t new_goal_count,
                                              const double dt)
{
    size_t next_via_idx = 0;
    const size_t via_size = via_points.size();

    if (via_size == 0) return;

    while (current_count <= via_points[next_via_idx].count && next_via_idx < via_size) ++next_via_idx;
    if (next_via_idx == via_size) {
        std::cerr << "[LimbTrajectoryGenerator] Current count is greater than goal count. Something wrong!!" << std::endl;
        return;
    }

    // TODO: via pointsに変更可能かどうかのフラグを立てる？ 変更すると干渉が生じそうなpointとかあるので
    // const hrp::Vector3 diff_pos = new_goal.translation() - via_points.back().point;
    // const size_t diff_count = (new_goal_count - via_points.back().count) / (via_size - next_via_idx);

    // // TODO: その足の初めの一歩はstartを変えないが，その次以降はstartも変わる
    // //       スレッド分けて最初の一歩以外はそこで計算するようにする？
    // //       そうすれば1からやっても良い気がする
    // for (size_t i = next_via_idx; i < via_size; ++i) {
    //     via_points[i].point += diff_pos;
    //     via_points[i].count += diff_count;
    // }
    // via_points.back().count = new_goal_count;

    if (traj_type == RECTANGLE) modifyRectangleViaPoints(new_goal.translation(), current_count, new_goal_count, dt);

    // TODO: 回転の補間がなめらかじゃ無さそう
    start_rot = current_coord.linear();
    diff_rot = Eigen::AngleAxisd(current_coord.linear().transpose() * new_goal.linear());
    via_points.back().diff_rot_angle = diff_rot.angle();
    diff_rot.angle() = 0;

    std::cerr << "modif via points:" << std::endl;
    for (auto&& point : via_points) {
        std::cerr << point.point.transpose() << ", count: " << point.count << std::endl;
    }
    std::cerr << "current_count: " << current_count << ", new_goal_count: " << new_goal_count << std::endl;
}

void LimbTrajectoryGenerator::modifyRectangleViaPoints(const hrp::Vector3& goal,
                                                       const size_t current_count,
                                                       const size_t new_goal_count,
                                                       const double dt)
{
    size_t via_idx;
    double remain_time;
    hrp::Vector3 target;
    double rot_angle;
    std::tie(via_idx, remain_time, target, rot_angle) = calcTarget(current_count, dt);

    const size_t via_size = via_points.size();
    const double height = via_idx == 2 ? target[2] : std::max(via_points[via_size - 2].point[2] - goal[2], via_points[via_size - 2].point[2] - via_points.back().point[2]);

    via_points[0].point = target;
    via_points[0].count = current_count;

    via_points[1].point = target;
    via_points[1].point[2] = goal[2] + height;

    via_points[2].point = goal;
    via_points[2].point[2] = goal[2] + height;

    via_points[3].point = goal;
    via_points[3].count = new_goal_count;

    double sum_distance = 0;
    for (size_t i = 0; i < 3; ++i) {
        sum_distance += (via_points[i + 1].point - via_points[i].point).squaredNorm();
    }
    const size_t diff_count = new_goal_count - current_count;

    via_points[1].count = via_points[0].count + static_cast<size_t>((via_points[1].point - via_points[0].point).squaredNorm() / sum_distance * diff_count);
    via_points[2].count = via_points[1].count + static_cast<size_t>((via_points[2].point - via_points[1].point).squaredNorm() / sum_distance * diff_count);
}

}
