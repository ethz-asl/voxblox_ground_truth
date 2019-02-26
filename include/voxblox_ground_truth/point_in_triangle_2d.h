//
// Created by victor on 26.02.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_POINT_IN_TRIANGLE_H
#define VOXBLOX_GROUND_TRUTH_POINT_IN_TRIANGLE_H

#include "voxblox_ground_truth/common.h"

int orientation(const Point2D &vertex_one, const Point2D &vertex_two,
                float &twice_signed_area) {
  twice_signed_area =
      vertex_one[1] * vertex_two[0] - vertex_one[0] * vertex_two[1];

  if(twice_signed_area > 0) return 1;
  else if(twice_signed_area < 0) return -1;
  else if(vertex_two[1] > vertex_one[1]) return 1;
  else if(vertex_two[1] < vertex_one[1]) return -1;
  else if(vertex_one[0] > vertex_two[0]) return 1;
  else if(vertex_one[0] < vertex_two[0]) return -1;
  else return 0; // only true when vertex_one[0]==vertex_two[0] and vertex_one[1]==vertex_two[1]
}

bool point_in_triangle_2d(const Point2D &point,
                          Point2D vertex_a, Point2D vertex_b, Point2D vertex_c,
                          Point *barycentric_coordinates) {
  vertex_a -= point;
  vertex_b -= point;
  vertex_c -= point;

  int signa = orientation(vertex_b, vertex_c, barycentric_coordinates->operator[](0));
  if(signa == 0) return false;

  int signb = orientation(vertex_c, vertex_a, barycentric_coordinates->operator[](1));
  if(signb != signa) return false;

  int signc = orientation(vertex_a, vertex_b, barycentric_coordinates->operator[](2));
  if(signc != signa) return false;

  // If the point is within the triangle,
  // the barymetric coordinates should never be zero
  double sum = barycentric_coordinates->sum();
  CHECK(sum != 0);

  barycentric_coordinates->operator[](0) /= sum;
  barycentric_coordinates->operator[](1) /= sum;
  barycentric_coordinates->operator[](2) /= sum;
  return true;
}

#endif //VOXBLOX_GROUND_TRUTH_POINT_IN_TRIANGLE_H
