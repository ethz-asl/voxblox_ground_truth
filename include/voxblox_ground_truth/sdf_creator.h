#ifndef VOXBLOX_GROUND_TRUTH_SDF_CREATOR_H_
#define VOXBLOX_GROUND_TRUTH_SDF_CREATOR_H_

#include <voxblox/core/tsdf_map.h>

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

  void integrateTriangle(
      const TriangularFaceVertexCoordinates& vertex_coordinates);

  const voxblox::TsdfMap& getTsdfMap();
  const voxblox::Layer<IntersectionVoxel>& getIntersectionLayer() {
    return intersection_layer_;
  }

  size_t getNumberOfAllocatedBlocks() {
    return tsdf_map_.getTsdfLayer().getNumberOfAllocatedBlocks();
  }

  bool getFillInside() { return fill_inside_; }
  void setFillInside(bool fill_inside) { fill_inside_ = fill_inside; }

  // Floodfill the unoccupied space in the mesh, up to the bounds of the AABB.
  // TODO(helenol): Allow specifying some other bounds?
  void floodfillUnoccupied(FloatingPoint distance_value);

 private:
  bool signs_up_to_date_;
  void updateSigns();
  void getAABBIndices(GlobalIndex* global_voxel_index_min,
                      GlobalIndex* global_voxel_index_max) const;

  voxblox::TsdfMap tsdf_map_;
  voxblox::Layer<IntersectionVoxel> intersection_layer_;

  const FloatingPoint voxel_size_;
  const FloatingPoint voxel_size_inv_;
  const IndexElement voxels_per_side_;
  const FloatingPoint voxels_per_side_inv_;

  // Whether to fill the *inside* of the object (think of Stanford Bunny) or
  // *outside* of the object (think of watertight mesh of the inside of an
  // office). For Gazebo inputs should be set to TRUE.
  bool fill_inside_ = false;
  const unsigned int aabb_padding_ = 1;
};
}  // namespace voxblox_ground_truth

#endif  // VOXBLOX_GROUND_TRUTH_SDF_CREATOR_H_
