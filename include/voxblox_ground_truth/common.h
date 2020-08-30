#ifndef VOXBLOX_GROUND_TRUTH_COMMON_H_
#define VOXBLOX_GROUND_TRUTH_COMMON_H_

#include <Eigen/StdVector>
#include <glog/logging.h>

typedef Eigen::Vector3f Point;
typedef Eigen::Vector2f Point2D;

// Voxel used to track the intersection counts
// NOTE: The intersection map is used to derive signs of the SDF map
struct IntersectionVoxel {
  unsigned int count = 0;
};

// IDs of the vertices for all three triangle corners
struct TriangularFaceVertexIds {
  uint32_t vertex_id_a;
  uint32_t vertex_id_b;
  uint32_t vertex_id_c;
};

// Coordinates of the vertices for all three triangle corners
struct TriangularFaceVertexCoordinates {
  Point vertex_a;
  Point vertex_b;
  Point vertex_c;
};

// Axis Aligned Bounding Box struct
struct AABB {
  Point min = {INFINITY, INFINITY, INFINITY};
  Point max = {-INFINITY, -INFINITY, -INFINITY};
  static AABB fromPoints(const Point& a, const Point& b, const Point& c) {
    AABB aabb;
    aabb.min = a.cwiseMin(b.cwiseMin(c));
    aabb.max = a.cwiseMax(b.cwiseMax(c));
    return aabb;
  }
};

inline bool visualizeIntersectionCount(const IntersectionVoxel& voxel,
                                       const Point& /*coord*/,
                                       double* intensity) {
  CHECK_NOTNULL(intensity);
  if (voxel.count > 0) {
    *intensity = voxel.count;
    return true;
  }
  return false;
}

#endif  // VOXBLOX_GROUND_TRUTH_COMMON_H_
