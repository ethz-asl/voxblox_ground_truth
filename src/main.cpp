//
// Created by victor on 13.02.19.
//
// Credits: - Local SDF generation and sign logic are inspired by
//            Christopher Batty's SDFGen (www.cs.columbia.edu/~batty)
//          - Ply importing is handled by ddiakopoulos's
//            tinyply (https://github.com/ddiakopoulos/tinyply)

#include <voxblox/core/common.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "voxblox_ground_truth/common.h"
#include "voxblox_ground_truth/io/tinyply.h"

int main(int argc, char* argv[]) {
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

  // Parse the file's data
  ply_file.read(ply_ss);
  std::cout << "-- total number of vertices " << ply_vertices->count
            << std::endl;
  std::cout << "-- total number of faces " << ply_faces->count << std::endl;

  // Cast vertices
  const size_t numVerticesBytes = ply_vertices->buffer.size_bytes();
  std::vector<Vertex> vertices(ply_vertices->count);
  std::memcpy(vertices.data(), ply_vertices->buffer.get(), numVerticesBytes);

  // Cast triangular faces
  const size_t numFacesBytes = ply_faces->buffer.size_bytes();
  std::vector<TriangularFace> triangles(ply_faces->count);
  std::memcpy(triangles.data(), ply_faces->buffer.get(), numFacesBytes);

  /* Initialize the TSDF */

  return 0;
}
