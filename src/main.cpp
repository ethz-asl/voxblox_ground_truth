//
// Created by victor on 13.02.19.
//

#include <voxblox/core/tsdf_map.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "voxblox_ground_truth/common.h"
#include "voxblox_ground_truth/distance_to_triangle.h"
#include "voxblox_ground_truth/io/tinyply.h"

int main(int argc, char* argv[]) {
  using voxblox::VoxelIndex;
  using voxblox::IndexElement;
  // TODO(victorr): Check if the args fit the right format
  // TODO(victorr): Implement --help

  /* Process the command line arguments */
  std::string ply_filepath(argv[1]);

  std::stringstream voxel_size_raw(argv[2]);
  float voxel_size;
  voxel_size_raw >> voxel_size;

  /* Parse the PLY file */
  std::cout << "Importing .ply file: " << ply_filepath << std::endl;
  std::ifstream ply_ss(ply_filepath, std::ios::binary);
  if (ply_ss.fail()) {
    throw std::runtime_error("ERROR: File could not be opened.");
  }

  // Instantiate tinyply and parse the file header
  tinyply::PlyFile ply_file;
  ply_file.parse_header(ply_ss);

  // Indicate which fields should be imported (vertices and faces)
  std::shared_ptr<tinyply::PlyData> ply_vertices, ply_faces;
  ply_vertices =
      ply_file.request_properties_from_element("vertex", {"x", "y", "z"});
  ply_faces =
      ply_file.request_properties_from_element("face", {"vertex_indices"}, 3);
  // NOTE: At the moment only triangular meshes are supported by this program.
  //       The included tinyply parser has therefore been modified to
  //       assert that all lists exactly fit their size hint. In this case
  //       this enforces that all faces have 3 vertices (i.e. are triangular).

  // Parse the file's data
  ply_file.read(ply_ss);
  std::cout << "-- total number of vertices " << ply_vertices->count
            << std::endl;
  std::cout << "-- total number of faces " << ply_faces->count << std::endl;

  // Cast vertices
  std::vector<Point> vertices(ply_vertices->count);
  const size_t numVerticesBytes = ply_vertices->buffer.size_bytes();
  std::memcpy(vertices.data(), ply_vertices->buffer.get(), numVerticesBytes);

  // Cast triangular faces
  std::vector<TriangularFace> triangles(ply_faces->count);
  const size_t numFacesBytes = ply_faces->buffer.size_bytes();
  std::memcpy(triangles.data(), ply_faces->buffer.get(), numFacesBytes);

  /* Initialize the TSDF */
  voxblox::TsdfMap::Config config;
  config.tsdf_voxel_size = voxel_size;
  voxblox::TsdfMap tsdf_map(config);
  voxblox::FloatingPoint voxel_size_inv =
      tsdf_map.getTsdfLayer().voxel_size_inv();

  /* Generate the TSDF */
  for (const TriangularFace &triangle : triangles) {
    const Point &vertex_a = vertices[triangle.vertex_id_a];
    const Point &vertex_b = vertices[triangle.vertex_id_b];
    const Point &vertex_c = vertices[triangle.vertex_id_c];

    AABB aabb_tight = AABB::fromPoints(vertex_a, vertex_b, vertex_c);
    voxblox::Point aabb_padded_min = aabb_tight.min.array() - voxel_size;
    voxblox::Point aabb_padded_max = aabb_tight.max.array() + voxel_size;

    VoxelIndex voxel_index = voxblox::getGridIndexFromPoint<VoxelIndex>(
        aabb_padded_min, voxel_size_inv);
    VoxelIndex voxel_index_max = voxblox::getGridIndexFromPoint<VoxelIndex>(
        aabb_padded_max, voxel_size_inv);

    for (; voxel_index[0] < voxel_index_max[0]; voxel_index[0]++)
      for (; voxel_index[1] < voxel_index_max[1]; voxel_index[1]++)
        for (; voxel_index[2] < voxel_index_max[2]; voxel_index[2]++) {
          // Compute distance to triangle
          // TODO(victorr): Check if voxblox stores the distances
          //                at the voxel origins or at the voxel centers
          Point voxel_point =
              voxblox::getOriginPointFromGridIndex(voxel_index, voxel_size);
          float distance = distance_point_to_triangle(voxel_point, vertex_a,
                                                      vertex_b, vertex_c);
          std::cout << distance << std::endl;

          // TODO(victorr): Update voxel if new distance is lower
        }
  }

  return 0;
}
