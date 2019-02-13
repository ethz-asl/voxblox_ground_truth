//
// Created by victor on 13.02.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_COMMON_H_
#define VOXBLOX_GROUND_TRUTH_COMMON_H_

typedef Eigen::Vector3f Point;

struct TriangularFace {
  // IDs of the vertices for all three triangle corners
  uint32_t vertex_id_a;
  uint32_t vertex_id_b;
  uint32_t vertex_id_c;
};

struct AABB {
  Point min = {INFINITY, INFINITY, INFINITY};
  Point max = {-INFINITY, -INFINITY, -INFINITY};
  static AABB fromPoints(const Point &a, const Point &b, const Point &c) {
    AABB aabb;
    aabb.min = a.cwiseMin(b.cwiseMin(c));
    aabb.max = a.cwiseMax(b.cwiseMax(c));
    return aabb;
  }
};

#endif  // VOXBLOX_GROUND_TRUTH_COMMON_H_
