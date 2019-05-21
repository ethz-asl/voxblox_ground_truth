//
// Created by victor on 13.02.19.
//

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "voxblox_ground_truth/common.h"
#include "voxblox_ground_truth/io/tinyply.h"
#include "voxblox_ground_truth/sdf_creator.h"

// For debugging only
#include <ros/ros.h>
#include "voxblox_ground_truth/sdf_visualizer.h"

// TODO(victorr): Move out all the debugging visuals and
//                make this a proper command line tool
int main(int argc, char *argv[]) {
  using PointcloudMsg = pcl::PointCloud<pcl::PointXYZI>;
  // TODO(victorr): Implement args format check
  // TODO(victorr): Implement --help

  /* Advertise the debugging ROS topic */
  // Register with ROS
  ros::init(argc, argv, "voxblox_ground_truth");
  ros::NodeHandle nh_private("~");
  // Advertise the topics to visualize the SDF map in Rviz
  voxblox_ground_truth::SdfVisualizer sdf_visualizer(nh_private);

  /* Process the command line arguments */
  std::string ply_filepath(argv[1]);
  std::stringstream voxel_size_raw(argv[2]);
  float voxel_size;
  voxel_size_raw >> voxel_size;

  // TODO(victorr): Read the Transform from params as X Y Z Qx Qy Qz Qw
  // Transform the .ply into the right reference frame
  voxblox::Transformation transform;
  voxblox::Transformation::Vector3 scaled_axis_angle(3.14 / 2.0, 0, 0);
  transform.getRotation() = voxblox::Rotation(scaled_axis_angle);
  double scale_factor = 100;

  // TODO(victorr): Use PCL .ply mesh parser instead
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
  std::vector<TriangularFaceVertexIds> triangles(ply_faces->count);
  const size_t numFacesBytes = ply_faces->buffer.size_bytes();
  std::memcpy(triangles.data(), ply_faces->buffer.get(), numFacesBytes);

  // Initialize the SDF creator
  std::cout << "Generating the TSDF" << std::endl;
  voxblox::TsdfMap::Config map_config;
  map_config.tsdf_voxel_size = voxel_size;
  voxblox_ground_truth::SdfCreator sdf_creator(map_config);

  // Iterate over all triangles
  size_t triangle_i = 0;
  for (const TriangularFaceVertexIds &triangle : triangles) {
    // Indicate progress
    triangle_i++;
    // Only print progress for each promile of completion, to reduce IO wait
    if (triangle_i % (triangles.size() / 1000) == 0) {
      printf("\rProgress: %3.1f%% - total nr of blocks %lu",
             triangle_i / static_cast<double>(triangles.size()) * 100,
             sdf_creator.getNumberOfAllocatedBlocks());
      std::cout << std::flush;
    }

    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      std::cout << "\nShutting down..." << std::endl;
      return -1;
    }

    // Get the triangle vertices
    TriangularFaceVertexCoordinates triangle_vertices;
    triangle_vertices.vertex_a =
        scale_factor * (transform * vertices[triangle.vertex_id_a]);
    triangle_vertices.vertex_b =
        scale_factor * (transform * vertices[triangle.vertex_id_b]);
    triangle_vertices.vertex_c =
        scale_factor * (transform * vertices[triangle.vertex_id_c]);

    // Update the SDF with the new triangle
    sdf_creator.integrateTriangle(triangle_vertices);
  }
  std::cout << "\nDistance field building complete." << std::endl;

  /* Publish debugging visuals */
  bool publish_debug_visuals = false;
  nh_private.param("publish_visuals", publish_debug_visuals,
                   publish_debug_visuals);
  if (publish_debug_visuals) {
    std::cout << "Publishing visuals" << std::endl;
    sdf_visualizer.publishTsdfVisuals(
        sdf_creator.getTsdfMap().getTsdfLayer());
    sdf_visualizer.publishIntersectionVisuals(
        sdf_creator.getIntersectionLayer());
  }

  std::cout << "Done" << std::endl;
  ros::spin();

  return 0;
}
