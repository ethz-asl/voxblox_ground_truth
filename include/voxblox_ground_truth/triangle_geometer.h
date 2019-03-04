//
// Created by victor on 04.03.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_TRIANGLE_GEOMETER_H_
#define VOXBLOX_GROUND_TRUTH_TRIANGLE_GEOMETER_H_

#include "voxblox_ground_truth/common.h"

namespace voxblox_ground_truth {
class TriangleGeometer {
 public:
  // TODO(victorr): Make this method non-static and have the instance
  //                keep track of the vertices
  static float distance_point_to_triangle(const Point &point,
                                          const Point &vertex_a,
                                          const Point &vertex_b,
                                          const Point &vertex_c);

  static bool point_in_triangle_2d(const Point2D &point, Point2D vertex_a,
                                   Point2D vertex_b, Point2D vertex_c,
                                   Point *barycentric_coordinates);

 private:
  // TODO(victorr): Give this method a better name
  static int orientation(const Point2D &vertex_one, const Point2D &vertex_two,
                         float *twice_signed_area);
};
}  // namespace voxblox_ground_truth

#endif  // VOXBLOX_GROUND_TRUTH_TRIANGLE_GEOMETER_H_
