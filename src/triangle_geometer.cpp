//
// Created by victor on 04.03.19.
//

#include "voxblox_ground_truth/triangle_geometer.h"

namespace voxblox_ground_truth {
// This function is heavily based on the example on page 141 of:
// "Real-time collision detection" by Christer Ericson
float TriangleGeometer::distance_point_to_triangle(const Point &point,
                                                   const Point &vertex_a,
                                                   const Point &vertex_b,
                                                   const Point &vertex_c) {
  typedef Point Vector;

  // Check if the point is in the region outside A
  Vector ab = vertex_b - vertex_a;
  Vector ac = vertex_c - vertex_a;
  Vector ap = point - vertex_a;
  float d1 = ab.dot(ap);
  float d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    // The barycentric coordinates are (1,0,0) => the closest point is vertex_a
    return (vertex_a - point).norm();
  }

  // Check if P in vertex region outside B
  Vector bp = point - vertex_b;
  float d3 = ab.dot(bp);
  float d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) {
    // The barycentric coordinates are (0,1,0) => the closest point is vertex_b
    return (vertex_b - point).norm();
  }

  // Check if P in edge region of AB, if so return projection of P onto AB
  float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    float v = d1 / (d1 - d3);
    // The barycentric coordinates are (1-v,v,0)
    Point closest_pt = vertex_a + v * ab;
    return (closest_pt - point).norm();
  }

  // Check if P in vertex region outside C
  Vector cp = point - vertex_c;
  float d5 = ab.dot(cp);
  float d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) {
    // The barycentric coordinates are (0,0,1) => the closest point is vertex_c
    return (vertex_c - point).norm();
  }

  // Check if P in edge region of AC, if so return projection of P onto AC
  float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    float w = d2 / (d2 - d6);
    // The barycentric coordinates are (1-w,0,w)
    Point closest_pt = vertex_a + w * ac;
    return (closest_pt - point).norm();
  }

  // Check if P in edge region of BC, if so return projection of P onto BC
  float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    // The barycentric coordinates are (0,1-w,w)
    Point closest_pt = vertex_b + w * (vertex_c - vertex_b);
    return (closest_pt - point).norm();
  }

  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  float denom = 1.0f / (va + vb + vc);
  float v = vb * denom;
  float w = vc * denom;
  // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
  Point closest_pt = vertex_a + ab * v + ac * w;
  return (closest_pt - point).norm();
}

bool TriangleGeometer::point_in_triangle_2d(const Point2D &point,
                                            Point2D vertex_a, Point2D vertex_b,
                                            Point2D vertex_c,
                                            Point *barycentric_coordinates) {
  vertex_a -= point;
  vertex_b -= point;
  vertex_c -= point;

  // TODO(victorr): Use ->at(0) notation instead of ->operator[](0)
  int sign_a =
      orientation(vertex_b, vertex_c, &barycentric_coordinates->operator[](0));
  if (sign_a == 0) return false;

  int sign_b =
      orientation(vertex_c, vertex_a, &barycentric_coordinates->operator[](1));
  if (sign_b != sign_a) return false;

  int sign_c =
      orientation(vertex_a, vertex_b, &barycentric_coordinates->operator[](2));
  if (sign_c != sign_a) return false;

  // If the point is within the triangle,
  // the barymetric coordinates should never be zero
  double sum = barycentric_coordinates->sum();
  CHECK_NE(sum, 0);

  // TODO(victorr): Use ->at(0) notation instead of ->operator[](0)
  barycentric_coordinates->operator[](0) /= sum;
  barycentric_coordinates->operator[](1) /= sum;
  barycentric_coordinates->operator[](2) /= sum;
  return true;
}

int TriangleGeometer::orientation(const Point2D &vertex_one,
                                  const Point2D &vertex_two,
                                  float *twice_signed_area) {
  CHECK_NOTNULL(twice_signed_area);

  *twice_signed_area =
      vertex_one[1] * vertex_two[0] - vertex_one[0] * vertex_two[1];

  if (*twice_signed_area > 0)
    return 1;
  else if (*twice_signed_area < 0)
    return -1;
  else if (vertex_two[1] > vertex_one[1])
    return 1;
  else if (vertex_two[1] < vertex_one[1])
    return -1;
  else if (vertex_one[0] > vertex_two[0])
    return 1;
  else if (vertex_one[0] < vertex_two[0])
    return -1;
  else
    return 0;  // only true when vertex_one[0]==vertex_two[0]
               // and vertex_one[1]==vertex_two[1]
}
}  // namespace voxblox_ground_truth
