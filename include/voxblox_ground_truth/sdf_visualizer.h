#ifndef VOXBLOX_GROUND_TRUTH_SDF_VISUALIZER_H_
#define VOXBLOX_GROUND_TRUTH_SDF_VISUALIZER_H_

#include <ros/ros.h>
#include <voxblox_ros/ptcloud_vis.h>

#include "voxblox_ground_truth/common.h"

namespace voxblox_ground_truth {
class SdfVisualizer {
 public:
  explicit SdfVisualizer(ros::NodeHandle* nh_private);

  void publishIntersectionVisuals(
      const voxblox::Layer<IntersectionVoxel>& intersection_layer);

  void publishTsdfVisuals(const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer);

 private:
  ros::Publisher tsdf_map_pub_;
  ros::Publisher tsdf_map_surface_pub_;
  ros::Publisher tsdf_slice_pub_;
  ros::Publisher intersection_count_pub_;

  double tsdf_slice_height_;
};
}  // namespace voxblox_ground_truth

#endif  // VOXBLOX_GROUND_TRUTH_SDF_VISUALIZER_H_
