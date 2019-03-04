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
#include <voxblox_ros/ptcloud_vis.h>

// TODO(victorr): Move out all the debugging visuals and
//                make this a proper command line tool
int main(int argc, char *argv[]) {
  namespace ph = std::placeholders;
  using PointcloudMsg = pcl::PointCloud<pcl::PointXYZI>;
  // TODO(victorr): Implement args format check
  // TODO(victorr): Implement --help

  // TODO(victorr): Move the visuals to a separate class
  /* Advertise the debugging ROS topic */
  // Register with ROS
  ros::init(argc, argv, "voxgraph_odometry_simulator");
  ros::NodeHandle nh_private("~");
  // Advertise the topics to visualize the SDF map in Rviz
  ros::Publisher tsdf_map_pub =
      nh_private.advertise<PointcloudMsg>("tsdf_map", 1, true);
  ros::Publisher tsdf_slice_pub =
      nh_private.advertise<PointcloudMsg>("tsdf_slice", 1, true);
  ros::Publisher intersection_count_pub =
      nh_private.advertise<PointcloudMsg>("intersection_counts", 1, true);

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

  // Initialize the SDF creator
  std::cout << "Generating the TSDF" << std::endl;
  voxblox::TsdfMap::Config map_config;
  map_config.tsdf_voxel_size = voxel_size;
  voxblox_ground_truth::SdfCreator sdf_creator(map_config);

  // Transform the .ply into the right reference frame
  voxblox::Transformation transform;
  voxblox::Transformation::Vector3 scaled_axis_angle(3.14 / 2.0, 0, 0);
  transform.getRotation() = voxblox::Rotation(scaled_axis_angle);
  double scale_factor = 100;

  // Iterate over all triangles
  size_t triangle_i = 0;
  for (const TriangularFace &triangle : triangles) {
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
    const Point &vertex_a =
        scale_factor * (transform * vertices[triangle.vertex_id_a]);
    const Point &vertex_b =
        scale_factor * (transform * vertices[triangle.vertex_id_b]);
    const Point &vertex_c =
        scale_factor * (transform * vertices[triangle.vertex_id_c]);

    // Update the SDF with the new triangle
    sdf_creator.addTriangle(vertex_a, vertex_b, vertex_c);
  }
  std::cout << "\nDistance field building complete." << std::endl;

  /* Publish debugging visuals */
  std::cout << "Publishing visuals" << std::endl;
  PointcloudMsg tsdf_map_ptcloud_msg;
  PointcloudMsg tsdf_slice_ptcloud_msg;
  PointcloudMsg intersection_count_msg;
  tsdf_map_ptcloud_msg.header.frame_id = "world";
  tsdf_slice_ptcloud_msg.header.frame_id = "world";
  intersection_count_msg.header.frame_id = "world";
  voxblox::createDistancePointcloudFromTsdfLayer(
      sdf_creator.getTsdfMap().getTsdfLayer(), &tsdf_map_ptcloud_msg);
  voxblox::createDistancePointcloudFromTsdfLayerSlice(
      sdf_creator.getTsdfMap().getTsdfLayer(), 2, 8, &tsdf_slice_ptcloud_msg);
  voxblox::createColorPointcloudFromLayer<IntersectionVoxel>(
      sdf_creator.getIntersectionLayer(), &visualizeIntersectionCount,
      &intersection_count_msg);
  tsdf_map_pub.publish(tsdf_map_ptcloud_msg);
  tsdf_slice_pub.publish(tsdf_slice_ptcloud_msg);
  intersection_count_pub.publish(intersection_count_msg);

  std::cout << "Done" << std::endl;
  ros::spin();

  return 0;
}
