//
// Created by victor on 04.03.19.
//

#include <limits>
#include <ros/ros.h>
#include "voxblox_ground_truth/sdf_creator.h"
#include "voxblox_ground_truth/triangle_geometer.h"

namespace voxblox_ground_truth {
SdfCreator::SdfCreator(voxblox::TsdfMap::Config map_config)
    : tsdf_map_(map_config),
      voxel_size_(map_config.tsdf_voxel_size),
      voxel_size_inv_(1.0f / map_config.tsdf_voxel_size),
      voxels_per_side_(
          static_cast<IndexElement>(map_config.tsdf_voxels_per_side)),
      voxels_per_side_inv_(
          1.0f / static_cast<FloatingPoint>(map_config.tsdf_voxels_per_side)),
      intersection_layer_(map_config.tsdf_voxel_size,
                          map_config.tsdf_voxels_per_side),
      signs_up_to_date_(false) {}

void SdfCreator::integrateTriangle(
    const TriangularFaceVertexCoordinates &vertex_coordinates) {
  // Indicate that the computed signs are no longer valid
  signs_up_to_date_ = false;

  // Instantiate the triangle geometry tool
  const TriangleGeometer triangle_geometer(vertex_coordinates);

  // Get the triangle's Axis Aligned Bounding Box
  const AABB aabb_tight = triangle_geometer.getAABB();

  // Express the AABB corners in voxel index units
  GlobalIndex aabb_min_index = (aabb_tight.min.array() * voxel_size_inv_)
                                   .floor()
                                   .cast<LongIndexElement>();
  GlobalIndex aabb_max_index = (aabb_tight.max.array() * voxel_size_inv_)
                                   .ceil()
                                   .cast<LongIndexElement>();

  // Add padding to the AABB indices
  const GlobalIndex voxel_index_min = aabb_min_index.array() - aabb_padding_;
  const GlobalIndex voxel_index_max = aabb_max_index.array() + aabb_padding_;

  // Iterate over all voxels within the triangle's padded AABB
  LongIndexElement x, y, z;
  Point voxel_origin;
  for (z = voxel_index_min[2]; z < voxel_index_max[2]; z++) {
    for (y = voxel_index_min[1]; y < voxel_index_max[1]; y++) {
      for (x = voxel_index_min[0]; x < voxel_index_max[0]; x++) {
        GlobalIndex voxel_index(x, y, z);

        // Get the indices of the current voxel and its containing block
        BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
            voxel_index, voxels_per_side_inv_);
        VoxelIndex local_voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
            voxel_index, voxels_per_side_);

        // Allocate the block and get the voxel
        voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_map_.getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
        voxblox::TsdfVoxel &voxel =
            block_ptr->getVoxelByVoxelIndex(local_voxel_index);

        // Compute distance to triangle
        voxel_origin =
            voxblox::getOriginPointFromGridIndex(voxel_index, voxel_size_);
        float distance = triangle_geometer.getDistanceToPoint(voxel_origin);

        // Update voxel if new distance is lower or if it is new
        // TODO(victorr): Take the absolute distance, to account for signs that
        //                might already have been computed
        if (distance < voxel.distance || voxel.weight == 0.0f) {
          voxel.distance = distance;
          voxel.weight += 1;
        }
      }

      // Mark intersections
      // NOTE: We check if and where the ray that lies parallel to the x axis
      //       and goes through voxel origin collides with the triangle.
      //       If it does, we increase the intersection counter for the voxel
      //       containing the intersection.
      const Point2D ray(voxel_origin.y(), voxel_origin.z());
      Point barycentric_coordinates;
      bool ray_intersects_triangle =
          triangle_geometer.getRayIntersection(ray, &barycentric_coordinates);
      if (ray_intersects_triangle) {
        // Get the voxel x index at the intersection
        float intersection_x_coordinate =
            barycentric_coordinates[0] * vertex_coordinates.vertex_a[0] +
            barycentric_coordinates[1] * vertex_coordinates.vertex_b[0] +
            barycentric_coordinates[2] * vertex_coordinates.vertex_c[0];
        auto intersection_x_index = static_cast<IndexElement>(
            std::ceil(intersection_x_coordinate * voxel_size_inv_));

        // Get the indices of the corresponding voxel and its containing block
        BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
            GlobalIndex(intersection_x_index, y, z), voxels_per_side_inv_);
        VoxelIndex local_voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
            GlobalIndex(intersection_x_index, y, z), voxels_per_side_);

        // Allocate the block and get the voxel
        voxblox::Block<IntersectionVoxel>::Ptr block_ptr =
            intersection_layer_.allocateBlockPtrByIndex(block_index);
        IntersectionVoxel &intersection_voxel =
            block_ptr->getVoxelByVoxelIndex(local_voxel_index);

        // Increase the count of intersections for this grid cell
        intersection_voxel.count++;
      }
    }
  }
}

void SdfCreator::updateSigns() {
  LOG(INFO) << "Computing the signs...";

  // Get the TSDF AABB, expressed in voxel index units
  GlobalIndex global_voxel_index_min =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::max());
  GlobalIndex global_voxel_index_max =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::min());

  // Indices of each block's local axis aligned min and max corners
  const VoxelIndex local_voxel_index_min(0, 0, 0);
  const VoxelIndex local_voxel_index_max(voxels_per_side_, voxels_per_side_,
                                         voxels_per_side_);

  // Iterate over all allocated blocks in the map
  voxblox::BlockIndexList tsdf_block_list;
  tsdf_map_.getTsdfLayer().getAllAllocatedBlocks(&tsdf_block_list);
  for (const voxblox::BlockIndex &block_index : tsdf_block_list) {
    const GlobalIndex global_voxel_index_in_block_min =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_min, voxels_per_side_);
    const GlobalIndex global_voxel_index_in_block_max =
        voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
            block_index, local_voxel_index_max, voxels_per_side_);

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
        // Exit if CTRL+C was pressed
        if (!ros::ok()) {
          std::cout << "\nShutting down..." << std::endl;
          return;
        }

        GlobalIndex global_voxel_index(x, y, z);

        IntersectionVoxel *intersection_voxel =
            intersection_layer_.getVoxelPtrByGlobalIndex(global_voxel_index);

        if (intersection_voxel) {
          intersection_count += intersection_voxel->count;
        }

        // TODO(victorr): Set the + or - sign instead of flipping it,
        //                such that it's possible to recompute the sign
        if (intersection_count % 2 == 1) {
          // We're inside the surface
          voxblox::TsdfVoxel *tsdf_voxel =
              tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
                  global_voxel_index);
          if (tsdf_voxel) {
            tsdf_voxel->distance = -tsdf_voxel->distance;
          }
        }
      }
    }
  }

  // Indicate that the signs are now up to date
  signs_up_to_date_ = true;
  LOG(INFO) << "Computing signs completed.";
}

const voxblox::TsdfMap &SdfCreator::getTsdfMap() {
  // Compute the signs, unless they're already up to date
  if (!signs_up_to_date_) {
    updateSigns();
  }

  return tsdf_map_;
}

}  // namespace voxblox_ground_truth
