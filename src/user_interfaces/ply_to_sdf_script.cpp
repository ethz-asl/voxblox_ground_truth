//
// Created by victor on 13.02.19.
//

#include <pcl/io/ply_io.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "voxblox_ground_truth/common.h"
#include "voxblox_ground_truth/sdf_creator.h"

// For debugging only
#include <ros/ros.h>
#include "voxblox_ground_truth/sdf_visualizer.h"

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
  double scale_factor = 100;
  voxblox::Transformation transform;
  voxblox::Transformation::Vector3 scaled_axis_angle(3.14 / 2.0, 0, 0);
  transform.getRotation() = voxblox::Rotation(scaled_axis_angle);

  /* Load the PLY file */
  pcl::PolygonMesh mesh;
  std::cout << "Importing .ply file: " << ply_filepath << std::endl;
  pcl::io::loadPLYFile(ply_filepath, mesh);
  pcl::PointCloud<pcl::PointXYZ> vertex_coordinates;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_coordinates);

  // Initialize the SDF creator
  std::cout << "Generating the TSDF" << std::endl;
  voxblox::TsdfMap::Config map_config;
  map_config.tsdf_voxel_size = voxel_size;
  voxblox_ground_truth::SdfCreator sdf_creator(map_config);

  // Iterate over all triangles
  size_t triangle_i = 0;
  size_t num_triangles = mesh.polygons.size();
  for (const pcl::Vertices &polygon : mesh.polygons) {
    // Ensure that the polygon is a triangle (other meshes are not supported)
    CHECK_EQ(polygon.vertices.size(), 3);

    // Indicate progress
    triangle_i++;
    // Only print progress for each promile of completion, to reduce IO wait
    if (triangle_i % (num_triangles / 1000) == 0) {
      printf("\rProgress: %3.1f%% - total nr of blocks %lu",
             triangle_i / static_cast<double>(num_triangles) * 100,
             sdf_creator.getNumberOfAllocatedBlocks());
      std::cout << std::flush;
    }

    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      std::cout << "\nShutting down..." << std::endl;
      return -1;
    }

    // Extract the triangle's vertices from the vertex coordinate pointcloud
    const Point vertex_a =
        vertex_coordinates[polygon.vertices[0]].getVector3fMap();
    const Point vertex_b =
        vertex_coordinates[polygon.vertices[1]].getVector3fMap();
    const Point vertex_c =
        vertex_coordinates[polygon.vertices[2]].getVector3fMap();

    // Transform the vertices from mesh frame into world frame
    TriangularFaceVertexCoordinates triangle_vertices;
    triangle_vertices.vertex_a = scale_factor * (transform * vertex_a);
    triangle_vertices.vertex_b = scale_factor * (transform * vertex_b);
    triangle_vertices.vertex_c = scale_factor * (transform * vertex_c);

    // Update the SDF with the new triangle
    sdf_creator.integrateTriangle(triangle_vertices);
  }
  std::cout << "\nDistance field building complete." << std::endl;

  /* Publish debugging visuals */
  bool publish_debug_visuals = true;
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
