#ifndef VOXBLOX_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_
#define VOXBLOX_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <glog/logging.h>
#include <ros/ros.h>
#include <voxblox_msgs/FilePath.h>

#include "voxblox_ground_truth/sdf_visualizer.h"

namespace gazebo {
class VoxbloxGroundTruthPlugin : public WorldPlugin {
 public:
  VoxbloxGroundTruthPlugin();

  void Load(physics::WorldPtr world, sdf::ElementPtr _sdf) override;

  bool serviceCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT

 private:
  physics::WorldPtr world_;
  ros::NodeHandle nh_private_;
  voxblox::FloatingPoint voxel_size_;
  ros::ServiceServer srv_;

  const std::vector<std::string> mesh_type_names_ = {
      "POINTS", "LINES", "LINESTRIPS", "TRIANGLES", "TRIFANS", "TRISTRIPS"};

  const std::vector<std::string> mesh_file_extensions_ = {".dae", ".obj",
                                                          ".mtl"};

  voxblox_ground_truth::SdfVisualizer sdf_visualizer_;
};
}  // namespace gazebo

#endif  // VOXBLOX_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_
