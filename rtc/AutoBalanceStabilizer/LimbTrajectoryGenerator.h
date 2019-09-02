// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LimbTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef LIMBTRAJECTORYGENERATOR_H
#define LIMBTRAJECTORYGENERATOR_H

#include <vector>
#include "LinkConstraint.h"

namespace hrp {

struct ViaPoint
{
    hrp::Vector3 point;
    size_t count;
};

// TODO: limbごとの管理の仕方
//       LinkConstraintにふくめる？
//       Link IDで紐付けられたmap (unordered_map) ?
class LimbTrajectoryGenerator
{
  public:
    enum TrajectoryType : size_t {DELAY_HOFFARBIB = 0, CYCLOIDDELAY, RECTANGLE};

  private:
    TrajectoryType traj_type = CYCLOIDDELAY;
    std::vector<ViaPoint> via_points;

    hrp::Vector3 pos = hrp::Vector3::Zero();
    hrp::Vector3 vel = hrp::Vector3::Zero();
    hrp::Vector3 acc = hrp::Vector3::Zero();

    // Params for delay Hoff & Arbib
    double delay_time_offset = 0.2; // [s]

    // std::tuple<hrp::Vector3, hrp::Vector3, hrp::Vector3> calcDelayHoffArbibTrajectory(const size_t count);
    void calcDelayHoffArbibTrajectory(const size_t count, const double dt);

    void calcViaPoints(const TrajectoryType traj_type,
                       const hrp::Vector3& start, const hrp::Vector3& goal,
                       const size_t start_count, const size_t goal_count,
                       const double height);

    // Preset functions to generate via points for delay Hoff & Arbib trajectory
    void calcCycloidDelayViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                   const size_t start_count, const size_t goal_count,
                                   const double height);
    void calcRectangleViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                const size_t start_count, const size_t goal_count,
                                const double height);

  public:
    LimbTrajectoryGenerator() {}

    const std::vector<ViaPoint>& getViaPoints() const { return via_points; }
    void clearViaPoints() { via_points.clear(); }
    // TODO: posだけでなくrotも対応したい
    void calcViaPoints(const TrajectoryType traj_type,
                       std::vector<ConstraintsWithCount>& constraints_list, // TODO: constつけたいけどつけられない
                       const int link_id, const size_t count, const double height);

    /**
     * @fn
     * Calculate limb trajectory using via points and a current count
     * @return std::tuple<position, velocity, acceleration>
     */
    // std::tuple<hrp::Vector3, hrp::Vector3, hrp::Vector3> calcTrajectory(const size_t count);
    void calcTrajectory(const size_t count, const double dt);

    void setPos(const hrp::Vector3& _pos) { pos = _pos; }
    void setVel(const hrp::Vector3& _vel) { vel = _vel; }
    void setAcc(const hrp::Vector3& _acc) { acc = _acc; }
    const hrp::Vector3& getPos() const { return pos; }
    const hrp::Vector3& getVel() const { return vel; }
    const hrp::Vector3& getAcc() const { return acc; }

    void setDelayTimeOffset(const double offset) { delay_time_offset = offset; }
    double getDelayTimeOffset() const { return delay_time_offset; }
};

}

#endif // LIMBTRAJECTORYGENERATOR_H
