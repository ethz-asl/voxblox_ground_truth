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
#include "voxblox_ground_truth/point_in_triangle_2d.h"
#include "voxblox_ground_truth/io/tinyply.h"

// For debugging only
#include <ros/ros.h>
//#include <voxblox_msgs/Layer.h>
//#include <voxblox_ros/conversions.h>
#include <voxblox_ros/ptcloud_vis.h>

int main(int argc, char* argv[]) {
  using voxblox::VoxelIndex;
  using voxblox::GlobalIndex;
  using voxblox::BlockIndex;
  using voxblox::IndexElement;
  using voxblox::LongIndexElement;
  namespace ph = std::placeholders;
  using PointcloudMsg = pcl::PointCloud<pcl::PointXYZI>;
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

  /* Initialize the layers */
  voxblox::TsdfMap::Config config;
  config.tsdf_voxel_size = voxel_size;
  voxblox::TsdfMap tsdf_map(config);
  const voxblox::FloatingPoint voxel_size_inv =
      tsdf_map.getTsdfLayer().voxel_size_inv();
  const auto voxels_per_side =
      static_cast<IndexElement>(tsdf_map.getTsdfLayer().voxels_per_side());
  const voxblox::FloatingPoint voxels_per_side_inv =
      tsdf_map.getTsdfLayer().voxels_per_side_inv();
  voxblox::Layer<IntersectionVoxel> intersection_layer(
      voxel_size, config.tsdf_voxels_per_side);

  /* Generate the TSDF */
  std::cout << "Generating the TSDF" << std::endl;

  // Transform the .ply into the right reference frame
  voxblox::Transformation transform;
  voxblox::Transformation::Vector3 scaled_axis_angle(3.14/2.0,0,0);
  transform.getRotation() = voxblox::Rotation(scaled_axis_angle);
  double scale_factor = 100;

  // Iterate over all triangles
  size_t triangle_i = 0;
  for (const TriangularFace &triangle : triangles) {
    // Indicate progress
    triangle_i++;
    printf("\rProgress: %3.2f%% - total nr of blocks %lu",
           triangle_i / static_cast<double>(triangles.size()) * 100,
           tsdf_map.getTsdfLayer().getNumberOfAllocatedBlocks());

    // Exit if CTRL+C was pressed
    if (!ros::ok()) {
      std::cout << "\nShutting down..." << std::endl;
      break;
    }

    // Get the triangle vertices
    const Point &vertex_a =
        transform * (scale_factor * vertices[triangle.vertex_id_a]);
    const Point &vertex_b =
        transform * (scale_factor * vertices[triangle.vertex_id_b]);
    const Point &vertex_c =
        transform * (scale_factor * vertices[triangle.vertex_id_c]);

    // Get the triangle's Axis Aligned Bounding Box
    AABB aabb_tight = AABB::fromPoints(vertex_a, vertex_b, vertex_c);
    // Express the AABB corners in voxel index units
    GlobalIndex aabb_min_index =
        (aabb_tight.min.array() * voxel_size_inv).floor().cast<IndexElement>();
    GlobalIndex aabb_max_index =
        (aabb_tight.max.array() * voxel_size_inv).ceil().cast<IndexElement>();
    // Add padding
    const GlobalIndex voxel_index_min = aabb_min_index.array() - 1;
    const GlobalIndex voxel_index_max = aabb_max_index.array() + 1;

    // Iterate over all voxels within the triangle's padded AABB
    LongIndexElement x, y, z;
    Point voxel_origin;
    for (z = voxel_index_min[2]; z < voxel_index_max[2]; z++) {
      for (y = voxel_index_min[1]; y < voxel_index_max[1]; y++) {
        for (x = voxel_index_min[0]; x < voxel_index_max[0]; x++) {
          GlobalIndex voxel_index(x, y, z);

          // Compute distance to triangle
          // TODO(victorr): Check if voxblox stores the distances
          //                at the voxel origins or at the voxel centers
          voxel_origin =
              voxblox::getOriginPointFromGridIndex(voxel_index, voxel_size);
          float distance = distance_point_to_triangle(
              voxel_origin, vertex_a, vertex_b, vertex_c);

          // Get the indices of the corresponding voxel and its containing block
          BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
              voxel_index, voxels_per_side_inv);
          VoxelIndex local_voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
              voxel_index, voxels_per_side);

          // Allocate the block and get the voxel
          voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_map.getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
          voxblox::TsdfVoxel &voxel =
              block_ptr->getVoxelByVoxelIndex(local_voxel_index);

          // Update voxel if new distance is lower or if it is new
          if (distance < voxel.distance || voxel.weight == 0.0f) {
            voxel.distance = distance;
            voxel.weight = 1;
          }
        }

        // Mark intersections
        // TODO(victorr): Add proper explanation
        // Check if the ray parallel to x axis through voxel origin collides with the triangle
        // Note, since we don't care about x we can simply use the last voxel_origin
        Point barycentric_coordinates;
        if (point_in_triangle_2d(voxel_origin.tail<2>(), vertex_a.tail<2>(),
                                 vertex_b.tail<2>(), vertex_c.tail<2>(),
                                 &barycentric_coordinates)) {
          // Get the voxel x_index at the intersection
          float intersection_x_coordinate =
              barycentric_coordinates[0] * vertex_a[0]
              + barycentric_coordinates[1] * vertex_b[0]
              + barycentric_coordinates[2] * vertex_c[0];
          auto intersection_x_index = static_cast<IndexElement>(
              std::ceil(intersection_x_coordinate * voxel_size_inv));

          // Get the indices of the corresponding voxel and its containing block
          BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
              GlobalIndex(intersection_x_index, y, z), voxels_per_side_inv);
          VoxelIndex local_voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
              GlobalIndex(intersection_x_index, y, z), voxels_per_side);

          // Allocate the block and get the voxel
          voxblox::Block<IntersectionVoxel>::Ptr block_ptr =
              intersection_layer.allocateBlockPtrByIndex(block_index);
          IntersectionVoxel &intersection_voxel =
              block_ptr->getVoxelByVoxelIndex(local_voxel_index);

          // Increase the count of intersections for this grid cell
          intersection_voxel.count++;
        }
      }
    }
  }
  std::cout << "\nDistance field building complete."
            << "\nComputing the signs..."
            << std::endl;
  // Get the TSDF AABB, expressed in voxel index units
  GlobalIndex global_voxel_index_min =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::max());
  GlobalIndex global_voxel_index_max =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::min());

  // Indices of each block's local axis aligned min and max corners
  const VoxelIndex local_voxel_index_min(0, 0, 0);
  const VoxelIndex local_voxel_index_max(voxels_per_side, voxels_per_side,
                                         voxels_per_side);

  // Iterate over all allocated blocks in the submap
  voxblox::BlockIndexList tsdf_block_list;
  tsdf_map.getTsdfLayer().getAllAllocatedBlocks(&tsdf_block_list);
  for (const voxblox::BlockIndex &block_index : tsdf_block_list) {
    const GlobalIndex global_voxel_index_in_block_min =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_min, voxels_per_side);
    const GlobalIndex global_voxel_index_in_block_max =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_max, voxels_per_side);

    global_voxel_index_min =
        global_voxel_index_min.cwiseMin(global_voxel_index_in_block_min);
    global_voxel_index_max =
        global_voxel_index_max.cwiseMax(global_voxel_index_in_block_max);
  }

  // Iterate over all voxels within the TSDF's AABB
  LongIndexElement x, y, z;
  for (z = global_voxel_index_min[2]; z < global_voxel_index_max[2]; z++) {
    for (y = global_voxel_index_min[1]; y < global_voxel_index_max[1]; y++) {
      size_t intersection_count = 0;
      for (x = global_voxel_index_min[0]; x < global_voxel_index_max[0]; x++) {
        GlobalIndex global_voxel_index(x, y, z);

        IntersectionVoxel* intersection_voxel =
            intersection_layer.getVoxelPtrByGlobalIndex(global_voxel_index);

        if (intersection_voxel) {
          intersection_count += intersection_voxel->count;
        }
      }
    }
  }

  /* Publish debugging visuals */
  std::cout << "Publishing visuals" << std::endl;
  PointcloudMsg tsdf_map_ptcloud_msg;
  PointcloudMsg tsdf_slice_ptcloud_msg;
  PointcloudMsg intersection_count_msg;
  tsdf_map_ptcloud_msg.header.frame_id = "world";
  tsdf_slice_ptcloud_msg.header.frame_id = "world";
  intersection_count_msg.header.frame_id = "world";
  voxblox::createDistancePointcloudFromTsdfLayer(
      tsdf_map.getTsdfLayer(), &tsdf_map_ptcloud_msg);
  voxblox::createDistancePointcloudFromTsdfLayerSlice(
      tsdf_map.getTsdfLayer(), 2, 8, &tsdf_slice_ptcloud_msg);
  voxblox::createColorPointcloudFromLayer<IntersectionVoxel>(
      intersection_layer,
      &visualizeIntersectionCount,
      &intersection_count_msg);
  tsdf_map_pub.publish(tsdf_map_ptcloud_msg);
  tsdf_slice_pub.publish(tsdf_slice_ptcloud_msg);
  intersection_count_pub.publish(intersection_count_msg);

  ros::spin();
  return 0;
}
