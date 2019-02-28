//
// Created by victor on 27.02.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_GAZEBO_PLUGIN_H
#define VOXBLOX_GROUND_TRUTH_GAZEBO_PLUGIN_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo {
class VoxbloxGroundTruthPlugin : public WorldPlugin {
 public:
  VoxbloxGroundTruthPlugin() : WorldPlugin(), node_handle_("~") {}

  void Load(physics::WorldPtr world, sdf::ElementPtr _sdf) override {
    world_ = world;

    std::string service_name = "get_tsdf";
    // TODO(victorr): Implement proper logging
    gzlog << "Advertising service: " << service_name << std::endl;
    srv_ = node_handle_.advertiseService(
        service_name, &VoxbloxGroundTruthPlugin::serviceCallback, this);
  }

  bool serviceCallback(std_srvs::Empty::Request &request,
                       std_srvs::Empty::Response &response) {
    for (const physics::ModelPtr &model : world_->GetModels()) {
      std::cout << model->GetName() << std::endl;
      for (const physics::LinkPtr &link : model->GetLinks()) {
        std::cout << "- " << link->GetName() << std::endl;
        std::cout << "-- collisions:" << std::endl;
        for (const physics::CollisionPtr &collision: link->GetCollisions()) {
          std::cout << "--- " << collision->GetName() << std::endl;

          common::MeshManager* mesh_manager = common::MeshManager::Instance();
          // TODO(victorr): Add CHECK_NOTNULL once GLog enabled
            std::cout << collision->GetSDF()->GetElement("geometry")->ToString("") << std::endl;
            std::cout << collision->GetWorldPose() << std::endl;

          // Call appropriate TSDF generation depending on shape
          if (collision->GetShape()->HasType(physics::Base::BOX_SHAPE)) {
            std::cout << "---- BOX_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::CYLINDER_SHAPE)) {
            std::cout << "---- CYLINDER_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::HEIGHTMAP_SHAPE)) {
            std::cout << "---- HEIGHTMAP_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::MAP_SHAPE)) {
            std::cout << "---- MAP_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::MULTIRAY_SHAPE)) {
            std::cout << "---- MULTIRAY_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::RAY_SHAPE)) {
            std::cout << "---- RAY_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::PLANE_SHAPE)) {
            std::cout << "---- PLANE_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::SPHERE_SHAPE)) {
            std::cout << "---- SPHERE_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::MESH_SHAPE)) {
            std::cout << "---- MESH_SHAPE" << std::endl;
          } else if (collision->GetShape()->HasType(physics::Base::POLYLINE_SHAPE)) {
            std::cout << "---- POLYLINE_SHAPE" << std::endl;
          }
        }
      }
    }

    // TODO(victorr): Publish the TSDF on a topic
  };

 private:
  physics::WorldPtr world_;
  ros::NodeHandle node_handle_;
  ros::ServiceServer srv_;
};
}  // namespace gazebo

#endif //VOXBLOX_GROUND_TRUTH_GAZEBO_PLUGIN_H
