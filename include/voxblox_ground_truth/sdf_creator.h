//
// Created by victor on 04.03.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_SDF_CREATOR_H_
#define VOXBLOX_GROUND_TRUTH_SDF_CREATOR_H_

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/ptcloud_vis.h>
#include "voxblox_ground_truth/common.h"

namespace voxblox_ground_truth {
class SdfCreator {
 public:
  using VoxelIndex = voxblox::VoxelIndex;
  using GlobalIndex = voxblox::GlobalIndex;
  using BlockIndex = voxblox::BlockIndex;
  using IndexElement = voxblox::IndexElement;
  using LongIndexElement = voxblox::LongIndexElement;
  using FloatingPoint = voxblox::FloatingPoint;

  explicit SdfCreator(voxblox::TsdfMap::Config map_config);

  void addTriangle(const Point &vertex_a, const Point &vertex_b,
                   const Point &vertex_c);

  const voxblox::TsdfMap &getTsdfMap();
  const voxblox::Layer<IntersectionVoxel> &getIntersectionLayer() {
    return intersection_layer_;
  }

  size_t getNumberOfAllocatedBlocks() {
    return tsdf_map_.getTsdfLayer().getNumberOfAllocatedBlocks();
  }

 private:
  bool signs_up_to_date_;
  void updateSigns();

  voxblox::TsdfMap tsdf_map_;
  voxblox::Layer<IntersectionVoxel> intersection_layer_;

  const FloatingPoint voxel_size_;
  const FloatingPoint voxel_size_inv_;
  const IndexElement voxels_per_side_;
  const FloatingPoint voxels_per_side_inv_;
};
}  // namespace voxblox_ground_truth

#endif  // VOXBLOX_GROUND_TRUTH_SDF_CREATOR_H_
