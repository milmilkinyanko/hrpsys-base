// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LimbTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef LIMBTRAJECTORYGENERATOR_H
#define LIMBTRAJECTORYGENERATOR_H

#include <vector>
#include <hrpUtil/EigenTypes.h>
// #include "LinkConstraint.h"

namespace hrp {

class LimbTrajectoryGenerator
{
  public:
    enum TrajectoryType : size_t {LINEAR, DELAY_HOFFARBIB, CYCLOIDDELAY, RECTANGLE};
    struct ViaPoint
    {
        hrp::Vector3 point;
        double diff_rot_angle; // TODO: Rotation Matrixではなくて問題ないかb
        size_t count;
    };

  private:
    TrajectoryType traj_type = CYCLOIDDELAY;
    std::vector<ViaPoint> via_points;

    hrp::Vector3 pos = hrp::Vector3::Zero();
    hrp::Vector3 vel = hrp::Vector3::Zero();
    hrp::Vector3 acc = hrp::Vector3::Zero();

    hrp::Matrix33 start_rot = hrp::Matrix33::Identity();
    Eigen::AngleAxisd diff_rot = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    hrp::Matrix33 rot = hrp::Matrix33::Identity();
    double rot_vel = 0;
    double rot_acc = 0;

    // Params for delay Hoff & Arbib
    double delay_time_offset = 0.2; // [s]

    // std::tuple<hrp::Vector3, hrp::Vector3, hrp::Vector3> calcDelayHoffArbibTrajectory(const size_t count);
    std::tuple<double, hrp::Vector3, double> calcTarget(const size_t count, const double dt);
    void calcLinearTrajectory(const size_t count, const double dt);
    void calcDelayHoffArbibTrajectory(const size_t count, const double dt);

    // Preset functions to generate via points for delay Hoff & Arbib trajectory
    void calcCycloidDelayViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                   const size_t start_count, const size_t goal_count,
                                   const double height);
    void calcRectangleViaPoints(const hrp::Vector3& start, const hrp::Vector3& goal,
                                const size_t start_count, const size_t goal_count,
                                const double height);
  public:
    void copyState(const LimbTrajectoryGenerator& ltg, const hrp::Vector3& move_pos = hrp::Vector3::Zero());
    bool isViaPointsEmpty() const { return via_points.empty(); }
    const std::vector<ViaPoint>& getViaPoints() const { return via_points; }
    void clearViaPoints() { via_points.clear(); }

    void calcViaPoints(const TrajectoryType traj_type,
                       const Eigen::Isometry3d& start, const Eigen::Isometry3d& goal,
                       const size_t start_count, const size_t goal_count,
                       const double height);

    void setViaPoints(const TrajectoryType _traj_type,
                      const Eigen::Isometry3d& start,
                      const Eigen::Isometry3d& goal,
                      const std::vector<ViaPoint>& _via_points);

    void calcRotationViaPoints(const TrajectoryType _traj_type,
                               const Eigen::Isometry3d& start,
                               const Eigen::Vector3d& rot_axis,
                               const double rot_angle,
                               const size_t start_count,
                               const size_t goal_count);

    /**
     * @fn
     * Calculate limb trajectory using via points and a current count
     */
    void calcTrajectory(const size_t count, const double dt);

    void setPos(const hrp::Vector3& _pos) { pos = _pos; }
    void setVel(const hrp::Vector3& _vel) { vel = _vel; }
    void setAcc(const hrp::Vector3& _acc) { acc = _acc; }
    void setRot(const hrp::Matrix33& _rot) { rot = _rot; }
    const hrp::Vector3& getPos() const { return pos; }
    const hrp::Vector3& getVel() const { return vel; }
    const hrp::Vector3& getAcc() const { return acc; }
    const hrp::Matrix33& getRot() const { return rot; }

    void setDelayTimeOffset(const double offset) { delay_time_offset = offset; }
    double getDelayTimeOffset() const { return delay_time_offset; }
};

}

#endif // LIMBTRAJECTORYGENERATOR_H
