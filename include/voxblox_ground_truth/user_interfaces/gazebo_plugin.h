//
// Created by victor on 27.02.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_
#define VOXBLOX_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_

#include <glog/logging.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <vector>
#include "voxblox_ground_truth/sdf_visualizer.h"

namespace gazebo {
class VoxbloxGroundTruthPlugin : public WorldPlugin {
 public:
  VoxbloxGroundTruthPlugin()
      : WorldPlugin(),
        nh_private_("~"),
        sdf_visualizer_(nh_private_) {}

  void Load(physics::WorldPtr world, sdf::ElementPtr _sdf) override;

  bool serviceCallback(std_srvs::Empty::Request &request,     // NOLINT
                       std_srvs::Empty::Response &response);  // NOLINT

 private:
  physics::WorldPtr world_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer srv_;

  std::vector<std::string> mesh_type_names_ = {
      "POINTS", "LINES", "LINESTRIPS", "TRIANGLES", "TRIFANS", "TRISTRIPS"};

  voxblox_ground_truth::SdfVisualizer sdf_visualizer_;
};
}  // namespace gazebo

#endif  // VOXBLOX_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_
