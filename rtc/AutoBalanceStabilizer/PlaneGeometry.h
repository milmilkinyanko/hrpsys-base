// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  PlaneGeometry.h
 * @brief
 * @date  $Date$
 */

#ifndef PLANEGEOMETRY_H
#define PLANEGEOMETRY_H

#include <vector>
#include <utility>
#include <tuple>
#include <Eigen/Core>

namespace hrp {

enum class ProjectedPointRegion {LEFT, MIDDLE, RIGHT};

inline double calcCrossProduct(const Eigen::Ref<const Eigen::Vector2d>& a,
                               const Eigen::Ref<const Eigen::Vector2d>& b,
                               const Eigen::Ref<const Eigen::Vector2d>& o)
{
    return (a(0) - o(0)) * (b(1) - o(1)) - (a(1) - o(1)) * (b(0) - o(0));
}

// Calculate 2D convex hull based on Andrew's algorithm
inline std::vector<Eigen::Vector2d> calcConvexHull(const std::vector<Eigen::Vector2d>& points)
{
    const size_t num_points = points.size();
    size_t num_verts = 0;

    std::vector<Eigen::Vector2d> convex_hull(2 * num_points);
    std::vector<Eigen::Vector2d> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(),
              [](const Eigen::Vector2d& lv, const Eigen::Vector2d& rv) {
                  return lv(0) < rv(0) || (lv(0) == rv(0) && lv(1) < rv(1));
              });

    for (size_t i = 0; i < num_points; convex_hull[num_verts++] = sorted_points[i++]) {
        while (num_verts >= 2 && calcCrossProduct(convex_hull[num_verts - 1], sorted_points[i], convex_hull[num_verts - 2]) <= 0) num_verts--;
    }

    for (size_t i = num_points - 2, j = num_verts + 1; i >= 0; convex_hull[num_verts++] = sorted_points[i--]) {
        while (num_verts >= j && calcCrossProduct(convex_hull[num_verts - 1], sorted_points[i], convex_hull[num_verts - 2]) <= 0) num_verts--;
    }

    convex_hull.resize(num_verts - 1);
    return convex_hull;
}

// Calculate closest point on line (p1 - p2)
inline ProjectedPointRegion calcProjectedPointOnLine(Eigen::Ref<Eigen::Vector2d> projected_p,
                                                     const Eigen::Ref<const Eigen::Vector2d>& target,
                                                     const Eigen::Ref<const Eigen::Vector2d>& p1,
                                                     const Eigen::Ref<const Eigen::Vector2d>& p2)
{
    const Eigen::Vector2d to_target = target - p2;
    const Eigen::Vector2d line = p1 - p2;
    const double line_norm = line.norm();

    if (line_norm == 0) {
        projected_p = p1;
        return ProjectedPointRegion::LEFT;
    }

    const double ratio = to_target.dot(line.normalized()) / line_norm;
    if (ratio < 0) {
        projected_p = p2;
        return ProjectedPointRegion::RIGHT;
    } else if (ratio > 1) {
        projected_p = p1;
        return ProjectedPointRegion::LEFT;
    } else {
        projected_p = p2 + ratio * line;
        return ProjectedPointRegion::MIDDLE;
    }
}

inline bool calcClosestBoundaryPoint(Eigen::Ref<Eigen::Vector2d> point,
                                     const std::vector<Eigen::Vector2d>& convex_hull,
                                     size_t& right_idx,
                                     size_t& left_idx)
{
    const size_t num_verts = convex_hull.size();
    Eigen::Vector2d cur_closest_point;

    for (size_t i = 0; i < num_verts; i++) {
        switch(calcProjectedPointOnLine(cur_closest_point, point, convex_hull[left_idx], convex_hull[right_idx])) {
          case ProjectedPointRegion::LEFT:
              right_idx = left_idx;
              left_idx = (left_idx + 1) % num_verts;
              if ((point - convex_hull[right_idx]).dot(convex_hull[left_idx] - convex_hull[right_idx]) <= 0) {
                  point = cur_closest_point;
                  return true;
              }
              break;
          case ProjectedPointRegion::RIGHT:
              left_idx = right_idx;
              right_idx = (right_idx - 1) % num_verts;
              if ((point - convex_hull[left_idx]).dot(convex_hull[right_idx] - convex_hull[left_idx]) <= 0) {
                  point = cur_closest_point;
                  return true;
              }
              break;
          case ProjectedPointRegion::MIDDLE:
          default:
              point = cur_closest_point;
              return true;
        }
    }
    return false;
}

inline std::pair<size_t, size_t> findVerticesContainingPoint(const std::vector<Eigen::Vector2d>& convex_hull,
                                                             const Eigen::Ref<const Eigen::Vector2d>& point,
                                                             const Eigen::Ref<const Eigen::Vector2d>& inner_p)
{
    const size_t num_verts = convex_hull.size();
    size_t v_a = 0;
    size_t v_b = num_verts;

    while (v_a + 1 < v_b) {
        const size_t v_c = (v_a + v_b) / 2;
        if (calcCrossProduct(convex_hull[v_a], convex_hull[v_c], inner_p) > 0) {
            if (calcCrossProduct(convex_hull[v_a], point, inner_p) > 0 && calcCrossProduct(convex_hull[v_c], point, inner_p) < 0) v_b = v_c;
            else v_a = v_c;
        } else {
            if (calcCrossProduct(convex_hull[v_a], point, inner_p) < 0 && calcCrossProduct(convex_hull[v_c], point, inner_p) > 0) v_a = v_c;
            else v_b = v_c;
        }
    }
    v_b %= num_verts;

    return std::make_pair(v_a, v_b);
}

inline bool isInsideConvexHull(const std::vector<Eigen::Vector2d>& convex_hull,
                               const Eigen::Ref<const Eigen::Vector2d>& point)
{
    // set any inner point and binary search two vertices(convex_hull[v_a], convex_hull[v_b]) between which p is.
    const size_t num_verts = convex_hull.size();
    const Eigen::Vector2d inner_p = (convex_hull[0] + convex_hull[num_verts / 3] + convex_hull[2 * num_verts / 3]) / 3.0;
    const std::pair<size_t, size_t> verts = findVerticesContainingPoint(convex_hull, point, inner_p);

    if (calcCrossProduct(convex_hull[verts.first], convex_hull[verts.second], point) < 0) return false;
    return true;
}

inline bool isInsideConvexHull(const std::vector<Eigen::Vector2d>& convex_hull,
                               const Eigen::Ref<const Eigen::Vector2d>& point,
                               const Eigen::Ref<const Eigen::Vector2d>& inner_p)
{
    const std::pair<size_t, size_t> verts = findVerticesContainingPoint(convex_hull, point, inner_p);
    if (calcCrossProduct(convex_hull[verts.first], convex_hull[verts.second], point) < 0) return false;
    return true;
}

inline bool isInsideConvexHull(const std::vector<Eigen::Vector2d>& convex_hull,
                               const Eigen::Ref<const Eigen::Vector2d>& point,
                               const Eigen::Ref<const Eigen::Vector2d>& inner_p,
                               size_t& v_a, size_t& v_b)
{
    std::tie(v_a, v_b) = findVerticesContainingPoint(convex_hull, point, inner_p);

    if (calcCrossProduct(convex_hull[v_a], convex_hull[v_b], point) < 0) return false;
    return true;
}

inline bool isIntersectingTwoLines(const Eigen::Ref<const Eigen::Vector2d>& pa_1,
                                   const Eigen::Ref<const Eigen::Vector2d>& pa_2,
                                   const Eigen::Ref<const Eigen::Vector2d>& pb_1,
                                   const Eigen::Ref<const Eigen::Vector2d>& pb_2,
                                   const double EPS = 1e-6)
{
    const auto zero_vec = Eigen::Vector2d::Zero();
    return abs(calcCrossProduct(pa_2 - pa_1, pb_2 - pb_1, zero_vec)) > EPS || // non-parallel
        abs(calcCrossProduct(pa_2 - pa_1, pb_1 - pa_1, zero_vec)) < EPS; // same line
}

inline bool calcIntersectionOfTwoLines(Eigen::Ref<Eigen::Vector2d> intersection,
                                       const Eigen::Ref<const Eigen::Vector2d>& pa_1,
                                       const Eigen::Ref<const Eigen::Vector2d>& pa_2,
                                       const Eigen::Ref<const Eigen::Vector2d>& pb_1,
                                       const Eigen::Ref<const Eigen::Vector2d>& pb_2,
                                       const double EPS = 1e-6)
{
    const bool is_intersecting = isIntersectingTwoLines(pa_1, pa_2, pb_1, pb_2, EPS);
    if (is_intersecting) {
        const Eigen::Vector2d line_a = pa_2 - pa_1;
        const auto zero_vec = Eigen::Vector2d::Zero();
        const double d1 = abs(calcCrossProduct(line_a, pb_1 - pa_1, zero_vec));
        const double d2 = abs(calcCrossProduct(line_a, pb_2 - pa_1, zero_vec));
        const double t = d1 / (d1 + d2);

        intersection = pb_1 + (pb_2 - pb_1) * t;
    }

    return is_intersecting;
}

inline bool calcIntersectionOfConvexHullWithLine(Eigen::Ref<Eigen::Vector2d> intersection,
                                                 const std::vector<Eigen::Vector2d>& convex_hull,
                                                 const Eigen::Ref<const Eigen::Vector2d>& p1,
                                                 const Eigen::Ref<const Eigen::Vector2d>& p2)
{
    size_t v_a, v_b;
    if (isInsideConvexHull(convex_hull, p1)) {
        if (isInsideConvexHull(convex_hull, p2, p1, v_a, v_b)) return false;
        return calcIntersectionOfTwoLines(intersection, p1, p2, convex_hull[v_a], convex_hull[v_b]);
    } else {
        if (!isInsideConvexHull(convex_hull, p2)) return false;
        isInsideConvexHull(convex_hull, p1, p2, v_a, v_b);
        return calcIntersectionOfTwoLines(intersection, p1, p2, convex_hull[v_a], convex_hull[v_b]);
    }

    return false;
}

} // end of namespace hrp

#endif // PLANEGEOMETRY_H
