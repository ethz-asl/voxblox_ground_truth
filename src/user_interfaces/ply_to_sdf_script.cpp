#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/ply_io.h>

#include "voxblox_ground_truth/common.h"
#include "voxblox_ground_truth/sdf_creator.h"

// For debugging only
#include <ros/ros.h>

#include "voxblox_ground_truth/sdf_visualizer.h"

int main(int argc, char* argv[]) {
  using PointcloudMsg = pcl::PointCloud<pcl::PointXYZI>;
  /* Advertise the debugging ROS topic */
  // Register with ROS
  ros::init(argc, argv, "voxblox_ground_truth");
  ros::NodeHandle nh_private("~");
  // Advertise the topics to visualize the SDF map in Rviz
  voxblox_ground_truth::SdfVisualizer sdf_visualizer(&nh_private);

  /* Process the command line arguments */
  if (argc != 12) {
    const std::string executable_path(argv[0]);
    const std::string executable_filename =
        executable_path.substr(executable_path.find_last_of('/') + 1);
    ROS_INFO_STREAM("Usage:\n"
                    << "./" << executable_filename << " "
                    << "[ply_input_file_path] [tsdf_output_file_path] "
                       "[voxel_size] [scale_factor] "
                       "[X] [Y] [Z] [Qx] [Qy] [Qz] [Qw]");
    return -1;
  }
  std::string ply_input_filepath(argv[1]);
  std::string tsdf_output_filepath(argv[2]);
  float voxel_size;
  std::stringstream(argv[3]) >> voxel_size;
  // Transformation and scaling from mesh to world frame
  double scale_factor;
  std::stringstream(argv[4]) >> scale_factor;
  voxblox::FloatingPoint x, y, z;
  std::stringstream(argv[5]) >> x;
  std::stringstream(argv[6]) >> y;
  std::stringstream(argv[7]) >> z;
  voxblox::FloatingPoint qx, qy, qz, qw;
  std::stringstream(argv[8]) >> qx;
  std::stringstream(argv[9]) >> qy;
  std::stringstream(argv[10]) >> qz;
  std::stringstream(argv[11]) >> qw;
  voxblox::Transformation transform;
  transform.getPosition() = voxblox::Transformation::Position(x, y, z);
  transform.getRotation().setValues(qx, qy, qz, qw);
  // NOTE: We don't check the quaternion's norm, since this is already done in
  //       Rotation::setValues(...)

  // Write the transformation params to cout for debugging
  Eigen::IOFormat ioformat(Eigen::StreamPrecision, Eigen::DontAlignCols, "; ",
                           "; ", "", "", "", "");
  ROS_INFO_STREAM(
      "Will apply transformation:\n"
      << "- scaling: " << scale_factor << "\n"
      << "- translation: " << transform.getPosition().format(ioformat) << "\n"
      << "- rotation: " << transform.getRotation().vector().format(ioformat));

  /* Load the PLY file */
  pcl::PolygonMesh mesh;
  ROS_INFO_STREAM("Importing .ply file: " << ply_input_filepath);
  pcl::io::loadPLYFile(ply_input_filepath, mesh);
  pcl::PointCloud<pcl::PointXYZ> vertex_coordinates;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_coordinates);

  // Initialize the SDF creator
  ROS_INFO("Generating the TSDF");
  voxblox::TsdfMap::Config map_config;
  map_config.tsdf_voxel_size = voxel_size;
  voxblox_ground_truth::SdfCreator sdf_creator(map_config);

  // Get parameters for whether inside the mesh is free or occupied.
  bool fill_inside = sdf_creator.getFillInside();
  nh_private.param("fill_inside", fill_inside, fill_inside);
  sdf_creator.setFillInside(fill_inside);

  // Iterate over all triangles
  size_t triangle_i = 0;
  size_t num_triangles = mesh.polygons.size();
  for (const pcl::Vertices& polygon : mesh.polygons) {
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
      ROS_INFO("Shutting down...");
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
    triangle_vertices.vertex_a = transform * (scale_factor * vertex_a);
    triangle_vertices.vertex_b = transform * (scale_factor * vertex_b);
    triangle_vertices.vertex_c = transform * (scale_factor * vertex_c);

    // Update the SDF with the new triangle
    sdf_creator.integrateTriangle(triangle_vertices);
  }
  ROS_INFO("Distance field building complete.");

  // Optionally floodfill unoccupied space.
  bool floodfill_unoccupied = false;
  nh_private.param("floodfill_unoccupied", floodfill_unoccupied,
                   floodfill_unoccupied);
  if (floodfill_unoccupied) {
    sdf_creator.floodfillUnoccupied(4 * voxel_size);
  }

  /* Publish debugging visuals */
  bool publish_debug_visuals = true;
  nh_private.param("publish_visuals", publish_debug_visuals,
                   publish_debug_visuals);
  if (publish_debug_visuals) {
    ROS_INFO("Publishing visuals");
    sdf_visualizer.publishTsdfVisuals(sdf_creator.getTsdfMap().getTsdfLayer());
    sdf_visualizer.publishIntersectionVisuals(
        sdf_creator.getIntersectionLayer());
  }

  // Save the TSDF to a file
  ROS_INFO_STREAM("Saving TSDF to file: " << tsdf_output_filepath);
  sdf_creator.getTsdfMap().getTsdfLayer().saveToFile(tsdf_output_filepath,
                                                     true);

  ROS_INFO("Done");
  ros::spin();

  return 0;
}
