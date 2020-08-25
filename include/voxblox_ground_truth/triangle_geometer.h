#ifndef VOXBLOX_GROUND_TRUTH_TRIANGLE_GEOMETER_H_
#define VOXBLOX_GROUND_TRUTH_TRIANGLE_GEOMETER_H_

#include <utility>

#include "voxblox_ground_truth/common.h"

namespace voxblox_ground_truth {
class TriangleGeometer {
 public:
  explicit TriangleGeometer(TriangularFaceVertexCoordinates vertex_coordinates)
      : vertices_(std::move(vertex_coordinates)) {}

  AABB getAABB() const;

  float getDistanceToPoint(const Point& point) const;

  bool getRayIntersection(const Point2D& ray_yz,
                          Point* barycentric_coordinates) const;

 private:
  const TriangularFaceVertexCoordinates vertices_;

  int getRelativeOrientation(const Point2D& vertex_one,
                             const Point2D& vertex_two,
                             float* twice_signed_area) const;
};
}  // namespace voxblox_ground_truth

#endif  // VOXBLOX_GROUND_TRUTH_TRIANGLE_GEOMETER_H_
