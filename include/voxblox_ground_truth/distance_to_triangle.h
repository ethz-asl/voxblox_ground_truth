//
// Created by victor on 17.02.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_DISTANCE_TO_TRIANGLE_H_
#define VOXBLOX_GROUND_TRUTH_DISTANCE_TO_TRIANGLE_H_

#include "voxblox_ground_truth/common.h"

// This function is heavily based on the example on page 141 of:
// "Real-time collision detection" by Christer Ericson
float distance_point_to_triangle(const Point &point, const Point &vertex_a,
                                 const Point &vertex_b, const Point &vertex_c) {
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

#endif  // VOXBLOX_GROUND_TRUTH_DISTANCE_TO_TRIANGLE_H_
