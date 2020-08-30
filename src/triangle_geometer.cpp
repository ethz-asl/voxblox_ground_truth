#include "voxblox_ground_truth/triangle_geometer.h"

namespace voxblox_ground_truth {
// This function is heavily based on the example on page 141 of:
// "Real-time collision detection" by Christer Ericson
float TriangleGeometer::getDistanceToPoint(const Point& point) const {
  using Vector = Point;

  // Check if the point is in the region outside A
  const Vector ab = vertices_.vertex_b - vertices_.vertex_a;
  const Vector ac = vertices_.vertex_c - vertices_.vertex_a;
  const Vector ap = point - vertices_.vertex_a;
  const float d1 = ab.dot(ap);
  const float d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    // The barycentric coordinates are (1,0,0) => the closest point is vertex_a
    return (vertices_.vertex_a - point).norm();
  }

  // Check if P in vertex region outside B
  const Vector bp = point - vertices_.vertex_b;
  const float d3 = ab.dot(bp);
  const float d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) {
    // The barycentric coordinates are (0,1,0) => the closest point is vertex_b
    return (vertices_.vertex_b - point).norm();
  }

  // Check if P in edge region of AB, if so return projection of P onto AB
  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float v = d1 / (d1 - d3);
    // The barycentric coordinates are (1-v,v,0)
    Point closest_pt = vertices_.vertex_a + v * ab;
    return (closest_pt - point).norm();
  }

  // Check if P in vertex region outside C
  Vector cp = point - vertices_.vertex_c;
  const float d5 = ab.dot(cp);
  const float d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) {
    // The barycentric coordinates are (0,0,1) => the closest point is vertex_c
    return (vertices_.vertex_c - point).norm();
  }

  // Check if P in edge region of AC, if so return projection of P onto AC
  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float w = d2 / (d2 - d6);
    // The barycentric coordinates are (1-w,0,w)
    const Point closest_pt = vertices_.vertex_a + w * ac;
    return (closest_pt - point).norm();
  }

  // Check if P in edge region of BC, if so return projection of P onto BC
  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    // The barycentric coordinates are (0,1-w,w)
    const Point closest_pt =
        vertices_.vertex_b + w * (vertices_.vertex_c - vertices_.vertex_b);
    return (closest_pt - point).norm();
  }

  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  const float denom = 1.0f / (va + vb + vc);
  const float v = vb * denom;
  const float w = vc * denom;
  // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
  const Point closest_pt = vertices_.vertex_a + ab * v + ac * w;
  return (closest_pt - point).norm();
}

AABB TriangleGeometer::getAABB() const {
  return AABB::fromPoints(vertices_.vertex_a, vertices_.vertex_b,
                          vertices_.vertex_c);
}

bool TriangleGeometer::getRayIntersection(
    const Point2D& ray_yz, Point* barycentric_coordinates) const {
  CHECK_NOTNULL(barycentric_coordinates);

  // Express the vertices A, B and C relative to the point
  const Point2D vertex_a_relative = vertices_.vertex_a.tail<2>() - ray_yz;
  const Point2D vertex_b_relative = vertices_.vertex_b.tail<2>() - ray_yz;
  const Point2D vertex_c_relative = vertices_.vertex_c.tail<2>() - ray_yz;

  // Check the orientation of B relative to C
  // NOTE: As a byproduct of the orientation checks, we also compute the signed
  //       areas. After being normalized, these correspond to the barycentric
  //       coordinates (see the end of this method).
  const int sign_a =
      getRelativeOrientation(vertex_b_relative, vertex_c_relative,
                             &barycentric_coordinates->operator[](0));
  // If the relative orientation is zero, vertices B and C must be equal.
  // This would mean that the triangle has no surface area and the ray therefore
  // cannot intersect it.
  if (sign_a == 0) return false;

  // Check the orientation of C relative to A
  const int sign_b =
      getRelativeOrientation(vertex_c_relative, vertex_a_relative,
                             &barycentric_coordinates->operator[](1));
  // If the signs differ, the solution to the intersection equation does not lie
  // inside the triangle (i.e. the ray does not intersect the triangle)
  if (sign_b != sign_a) return false;

  // Check the orientation of A relative to B
  const int sign_c =
      getRelativeOrientation(vertex_a_relative, vertex_b_relative,
                             &barycentric_coordinates->operator[](2));
  // If the signs differ, the solution to the intersection equation does not lie
  // inside the triangle (i.e. the ray does not intersect the triangle)
  if (sign_c != sign_a) return false;

  // If the point is within the triangle,
  // the barymetric coordinates should never be zero
  const double sum = barycentric_coordinates->sum();
  CHECK_NE(sum, 0);

  // Normalize the barycentric coordinates
  barycentric_coordinates->operator[](0) /= sum;
  barycentric_coordinates->operator[](1) /= sum;
  barycentric_coordinates->operator[](2) /= sum;

  return true;
}

int TriangleGeometer::getRelativeOrientation(const Point2D& vertex_one,
                                             const Point2D& vertex_two,
                                             float* twice_signed_area) const {
  CHECK_NOTNULL(twice_signed_area);

  // Compute the signed area (scaled by factor 2, but we don't care)
  *twice_signed_area =
      vertex_one[1] * vertex_two[0] - vertex_one[0] * vertex_two[1];

  // Return the relative orientation
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
    return 0;  // only true when both vertices are equal
}
}  // namespace voxblox_ground_truth
