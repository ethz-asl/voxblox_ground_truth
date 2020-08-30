#include "voxblox_ground_truth/sdf_visualizer.h"

namespace voxblox_ground_truth {
SdfVisualizer::SdfVisualizer(ros::NodeHandle* nh_private)
    : tsdf_slice_height_(0.0) {
  CHECK_NOTNULL(nh_private);

  // Get parameters.
  nh_private->param("tsdf_slice_height", tsdf_slice_height_,
                    tsdf_slice_height_);

  // Advertise the topics to visualize the SDF map in Rviz
  tsdf_map_pub_ = nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_map", 1, true);
  tsdf_map_surface_pub_ =
      nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>("tsdf_map_surface",
                                                             1, true);
  tsdf_slice_pub_ = nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_slice", 1, true);
  intersection_count_pub_ =
      nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "intersection_counts", 1, true);
}

void voxblox_ground_truth::SdfVisualizer::publishIntersectionVisuals(
    const voxblox::Layer<IntersectionVoxel>& intersection_layer) {
  LOG(INFO) << "Publishing intersection counts";
  pcl::PointCloud<pcl::PointXYZI> intersection_count_msg;
  intersection_count_msg.header.frame_id = "world";
  voxblox::createColorPointcloudFromLayer<IntersectionVoxel>(
      intersection_layer, &visualizeIntersectionCount, &intersection_count_msg);
  intersection_count_pub_.publish(intersection_count_msg);
}

void voxblox_ground_truth::SdfVisualizer::publishTsdfVisuals(
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer) {
  LOG(INFO) << "Publishing TSDF";
  pcl::PointCloud<pcl::PointXYZI> tsdf_map_ptcloud_msg;
  tsdf_map_ptcloud_msg.header.frame_id = "world";
  voxblox::createDistancePointcloudFromTsdfLayer(tsdf_layer,
                                                 &tsdf_map_ptcloud_msg);
  tsdf_map_pub_.publish(tsdf_map_ptcloud_msg);

  LOG(INFO) << "Publishing TSDF surface";
  pcl::PointCloud<pcl::PointXYZI> tsdf_map_surface_ptcloud_msg;
  tsdf_map_surface_ptcloud_msg.header.frame_id = "world";
  voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
      tsdf_layer, 0.1, &tsdf_map_surface_ptcloud_msg);
  tsdf_map_surface_pub_.publish(tsdf_map_surface_ptcloud_msg);

  LOG(INFO) << "Publishing TSDF slice";
  pcl::PointCloud<pcl::PointXYZI> tsdf_slice_ptcloud_msg;
  tsdf_slice_ptcloud_msg.header.frame_id = "world";
  voxblox::createDistancePointcloudFromTsdfLayerSlice(
      tsdf_layer, 2, tsdf_slice_height_, &tsdf_slice_ptcloud_msg);
  tsdf_slice_pub_.publish(tsdf_slice_ptcloud_msg);
}
}  // namespace voxblox_ground_truth
