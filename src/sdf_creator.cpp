#include <ros/ros.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox/utils/neighbor_tools.h>
#include <limits>

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
      signs_up_to_date_(false),
      fill_inside_(true) {}

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
        if (std::abs(distance) < std::abs(voxel.distance) ||
            voxel.weight == 0.0f) {
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

void SdfCreator::getAABBIndices(GlobalIndex *global_voxel_index_min,
                                GlobalIndex *global_voxel_index_max) const {
  *global_voxel_index_min =
      GlobalIndex::Constant(std::numeric_limits<LongIndexElement>::max());
  *global_voxel_index_max =
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

    *global_voxel_index_min =
        global_voxel_index_min->cwiseMin(global_voxel_index_in_block_min);
    *global_voxel_index_max =
        global_voxel_index_max->cwiseMax(global_voxel_index_in_block_max);
  }
}

void SdfCreator::updateSigns(bool dont_stop) {
  LOG(INFO) << "Computing the signs...";
  // Get the TSDF AABB, expressed in voxel index units
  GlobalIndex global_voxel_index_min, global_voxel_index_max;
  getAABBIndices(&global_voxel_index_min, &global_voxel_index_max);

  // Iterate over all voxels within the TSDF's AABB
  LongIndexElement x, y, z;
  for (z = global_voxel_index_min.z(); z < global_voxel_index_max.z(); z++) {
    for (y = global_voxel_index_min.y(); y < global_voxel_index_max.y(); y++) {
      size_t intersection_count = 0;
      for (x = global_voxel_index_min.x(); x < global_voxel_index_max.x();
           x++) {
        // Exit if CTRL+C was pressed
        if (!ros::ok() && !dont_stop) {
          std::cout << "\nShutting down..." << std::endl;
          return;
        }

        GlobalIndex global_voxel_index(x, y, z);

        IntersectionVoxel *intersection_voxel =
            intersection_layer_.getVoxelPtrByGlobalIndex(global_voxel_index);

        if (intersection_voxel) {
          intersection_count += intersection_voxel->count;
        }

        if (intersection_count % 2 == fill_inside_) {
          // We're inside the surface
          voxblox::TsdfVoxel *tsdf_voxel =
              tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
                  global_voxel_index);
          if (tsdf_voxel) {
            tsdf_voxel->distance = -std::abs(tsdf_voxel->distance);
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

void SdfCreator::floodfillUnoccupied(FloatingPoint distance_value,
                                     bool dont_stop) {
  // Get the TSDF AABB, expressed in voxel index units
  GlobalIndex global_voxel_index_min, global_voxel_index_max;
  getAABBIndices(&global_voxel_index_min, &global_voxel_index_max);

  // We're then going to iterate over all of the voxels within the AABB.
  // For any voxel which is unobserved and has free neighbors, mark it as free.
  // This assumes a watertight mesh (which is what's required for this algorithm
  // anyway).
  LongIndexElement x, y, z;
  // Iterate over x, y, z to minimize cache misses.
  for (x = global_voxel_index_min.x(); x < global_voxel_index_max.x(); x++) {
    for (y = global_voxel_index_min.y(); y < global_voxel_index_max.y(); y++) {
      for (z = global_voxel_index_min.z(); z < global_voxel_index_max.z();
           z++) {
        // Exit if CTRL+C was pressed
        if (!ros::ok() && !dont_stop) {
          std::cout << "\nShutting down..." << std::endl;
          return;
        }

        GlobalIndex global_voxel_index(x, y, z);
        voxblox::TsdfVoxel *tsdf_voxel =
            tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
                global_voxel_index);
        // If the block doesn't exist, make it.
        if (tsdf_voxel == nullptr) {
          BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(
              global_voxel_index, voxels_per_side_inv_);
          tsdf_map_.getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
          tsdf_voxel = tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
              global_voxel_index);
        }

        // If this is unobserved, then we need to check its neighbors.
        if (tsdf_voxel != nullptr && tsdf_voxel->weight <= 0.0f) {
          voxblox::GlobalIndexVector neighbors;
          voxblox::Neighborhood<>::IndexMatrix neighbor_indices;
          voxblox::Neighborhood<>::getFromGlobalIndex(global_voxel_index,
                                                      &neighbor_indices);
          for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
            const GlobalIndex &neighbor_index = neighbor_indices.col(idx);
            voxblox::TsdfVoxel *neighbor_voxel =
                tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
                    neighbor_index);
            // One free neighbor is enough to mark this voxel as free.
            if (neighbor_voxel != nullptr && neighbor_voxel->weight > 0.0f &&
                neighbor_voxel->distance > 0.0f) {
              tsdf_voxel->distance = distance_value;
              tsdf_voxel->weight = 1.0f;
              break;
            }
          }

          // If this voxel is in the AABB but didn't get updated from a
          // neighbor, assume it's outside the mesh and set the corresponding
          // sign of distance value.
          if (tsdf_voxel->weight <= 0.0f) {
            tsdf_voxel->weight = 1.0f;
            tsdf_voxel->distance =
                fill_inside_ ? distance_value : -distance_value;
          }
        }
        // Otherwise just move on, we're not modifying anything that's already
        // been updated.
      }
    }
  }

  if (!signs_up_to_date_) {
    updateSigns(dont_stop);
  }
}

void SdfCreator::clearSpaceInFrontOfTriangles(
    const std::vector<
        TriangularFaceVertexCoordinates,
        Eigen::aligned_allocator<TriangularFaceVertexCoordinates> >
        &triangle_list,
    const voxblox::Pointcloud &normals, FloatingPoint distance,
    float distance_value) {
  voxblox::LongIndexSet voxel_indices;

  CHECK_EQ(triangle_list.size(), normals.size());

  for (size_t i = 0; i < triangle_list.size(); i++) {
    const TriangularFaceVertexCoordinates &vertex_coordinates =
        triangle_list[i];
    Point normal = normals[i];

    // Compute the surface normal of the triangle.
    normal.normalize();

    // Get the voxel at the center of the triangle.
    Point center = (vertex_coordinates.vertex_a + vertex_coordinates.vertex_b +
                    vertex_coordinates.vertex_c) /
                   3.0;

    LongIndexElement distance_voxels = static_cast<LongIndexElement>(
        std::ceil(distance * tsdf_map_.getTsdfLayerPtr()->voxel_size_inv()));

    GlobalIndex center_index = voxblox::getGridIndexFromPoint<GlobalIndex>(
        center, tsdf_map_.getTsdfLayerPtr()->voxel_size_inv());

    // Get the AABB of the half-sphere (this is a little too big, but easier to
    // compute).
    GlobalIndex min_index = center_index.array() - distance_voxels;
    GlobalIndex max_index = center_index.array() + distance_voxels;

    // Iterate over x, y, z to minimize cache misses.
    GlobalIndex global_index = GlobalIndex::Zero();
    for (global_index.x() = min_index.x(); global_index.x() < max_index.x();
         global_index.x()++) {
      for (global_index.y() = min_index.y(); global_index.y() < max_index.y();
           global_index.y()++) {
        for (global_index.z() = min_index.z(); global_index.z() < max_index.z();
             global_index.z()++) {
          // Get the actual position of this point.
          Point current_point = voxblox::getCenterPointFromGridIndex(
              global_index, tsdf_map_.getTsdfLayerPtr()->voxel_size());

          // Project a half-sphere around the center, check if it's in front
          // of the half-sphere and not too far.
          FloatingPoint projected_distance =
              (current_point - center).dot(normal);
          if (projected_distance < 0 || projected_distance > distance) {
            continue;
          }
          voxel_indices.insert(global_index);
        }
      }
    }
  }

  // Iterate over every voxel.
  for (const GlobalIndex &voxel_global_index : voxel_indices) {
    // Get this voxel.
    voxblox::TsdfVoxel *tsdf_voxel =
        tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
            voxel_global_index);
    if (tsdf_voxel == nullptr) {
      // Make sure block is allocated.
      Point current_point = voxblox::getCenterPointFromGridIndex(
          voxel_global_index, tsdf_map_.getTsdfLayerPtr()->voxel_size());
      tsdf_map_.getTsdfLayerPtr()->allocateNewBlockByCoordinates(current_point);
      tsdf_voxel = tsdf_map_.getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
          voxel_global_index);
    }

    if (tsdf_voxel->weight <= 0.0f) {
      tsdf_voxel->weight = 1.0f;
      tsdf_voxel->distance = distance_value;
    }
  }
}

void SdfCreator::integrateTriangleWithNormal(
    const TriangularFaceVertexCoordinates &vertex_coordinates,
    const voxblox::Point &normal) {
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
        if (std::abs(distance) < std::abs(voxel.distance) ||
            voxel.weight == 0.0f) {
          voxel.distance = distance;
          voxel.weight += 1;

          // Correct the sign based on the normal.
          // This is pretty rough since we're just using the triangle center vs
          // voxel center comparison... Will have to see how it works.
          Point triangle_center;
          triangle_geometer.getClosestPoint(voxel_origin, &triangle_center);
          FloatingPoint projected_distance =
              (voxel_origin - triangle_center).dot(normal.normalized());
          // ROS_INFO_STREAM("Projected distance: " << projected_distance
          //                                       << " distance: " <<
          //                                       distance);
          if (projected_distance < 0) {
            voxel.distance = -voxel.distance;
          }
        }
      }
    }
  }
  signs_up_to_date_ = true;
}

}  // namespace voxblox_ground_truth
