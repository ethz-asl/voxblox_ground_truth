//
// Created by victor on 27.02.19.
//

#ifndef VOXBLOX_GROUND_TRUTH_GAZEBO_PLUGIN_H
#define VOXBLOX_GROUND_TRUTH_GAZEBO_PLUGIN_H

#include <glog/logging.h>
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
    gzlog << "Advertising service: " << service_name << std::endl;
    srv_ = node_handle_.advertiseService(
        service_name, &VoxbloxGroundTruthPlugin::serviceCallback, this);
  }

  bool serviceCallback(std_srvs::Empty::Request &request,
                       std_srvs::Empty::Response &response) {
    common::MeshManager* mesh_manager = common::MeshManager::Instance();
    CHECK_NOTNULL(mesh_manager);
    for (const physics::ModelPtr &model : world_->GetModels()) {
      for (const physics::LinkPtr &link : model->GetLinks()) {
        for (const physics::CollisionPtr &collision: link->GetCollisions()) {
          gzlog << "Processing '"<< collision->GetScopedName(true) << "'"
                << std::endl;

          msgs::Geometry geometry_msg;
          collision->GetShape()->FillMsg(geometry_msg);
          // TODO(victorr): Consider using the SDF directly, instead of the msg
//          sdf::ElementPtr geomElem = collision->GetShape()->GetSDF();

          gzlog << "------------------ SDF ------------------" << std::endl;
          gzlog << collision->GetShape()->GetSDF()->ToString("") << std::endl;
          gzlog << "-----------------------------------------" << std::endl;

          if (geometry_msg.has_type()) {
            std::string geometry_type_str =
                msgs::ConvertGeometryType(geometry_msg.type());

            if (geometry_type_str == "box" || geometry_type_str == "cylinder" ||
                geometry_type_str == "sphere" || geometry_type_str == "plane") {
              std::string mesh_name = "unit_" + geometry_type_str;
              if (mesh_manager) {
                const common::Mesh *mesh_ptr = mesh_manager->GetMesh(mesh_name);
                if (mesh_ptr) {
                  if (mesh_ptr->GetSubMeshCount() != 1) {
                    gzerr << "Encountered multiple sub meshes"
                          << "\nSkipping this mesh."
                          << std::endl;
                    continue;
                  }

                  // Create a copy of the submesh that can be manipulated
                  common::SubMesh submesh(mesh_ptr->GetSubMesh(0));

                  // Make sure we're dealing with a triangle mesh
                  if (submesh.GetPrimitiveType() !=
                      common::SubMesh::TRIANGLES) {
                    std::string mesh_type_str =
                        mesh_types_names_[submesh.GetPrimitiveType()];
                    gzerr << "Encountered a mesh with type "
                          << mesh_type_str << ". Currently, "
                          << "only triangular meshes are supported."
                          << "\nSkipping this mesh."
                          << std::endl;
                    continue;
                  }

                  // Find the mesh scale factor
                  // NOTE: The shape scale is absolute w.r.t. the world
                  ignition::math::Vector3d shape_scale =
                      collision->GetShape()->GetScale().Ign();
                  ignition::math::Vector3d geometry_size;
                  if (geometry_type_str == "box") {
                    geometry_size = msgs::ConvertIgn(geometry_msg.box().size());
                  } else if (geometry_type_str == "sphere") {
                    double radius = geometry_msg.sphere().radius();
                    geometry_size.Set(2.0 * radius, 2.0 * radius, 2.0 * radius);
                  } else if (geometry_type_str == "cylinder") {
                    double radius = geometry_msg.cylinder().radius();
                    double length = geometry_msg.cylinder().length();
                    geometry_size.Set(2.0 * radius, 2.0 * radius, length);
                  } else if (geometry_type_str == "plane") {
                    msgs::Vector2d dimensions = geometry_msg.plane().size();
                    geometry_size.Set(dimensions.x(), dimensions.y(), 1.0);
                  } else {
                    gzerr << "Could not get geometry size" << std::endl;
                    continue;
                  }
                  ignition::math::Vector3d scale_factor =
                      geometry_size * shape_scale;
                  gzlog << "Scale " << scale_factor << std::endl;

                  // Scale the mesh and transform it into world frame
                  const ignition::math::Pose3d transform =
                      collision->GetWorldPose().Ign();
                  for (unsigned int vertex_i = 0;
                       vertex_i < submesh.GetVertexCount(); vertex_i++) {
                    // Create a copy of the vertex to manipulate
                    ignition::math::Vector3d new_vertex =
                        submesh.Vertex(vertex_i);
                    // Scale it to the right size
                    new_vertex *= scale_factor;
                    // Transform it into world frame
                    new_vertex = transform.Rot() * new_vertex;
                    new_vertex += transform.Pos();
                    submesh.SetVertex(vertex_i, new_vertex);
                  }

                  // TODO(victorr): Plot the meshes in Gazebo (on another layer)
                  //                to make sure they overlap and are complete

                  unsigned int num_faces = submesh.GetIndexCount() / 3;
                  gzlog << "Integrating " << num_faces << " faces"
                        << std::endl;
                  for (unsigned int triangle_i = 0;
                       triangle_i < num_faces; triangle_i++) {
                    // TODO(victorr): Integrate TSDF here
                  }
                } else {
                  gzwarn << "Could not get pointer to mesh '"
                         << mesh_name << "'" << std::endl;
                }
              } else {
                gzwarn << "Could not get pointer to MeshManager" << std::endl;
              }
            } else {
              gzwarn << "Not yet able to process shapes of type: "
                     << geometry_type_str << std::endl;
              // TODO(victorr): Add support for Mesh shapes
              // physics::Base::MESH_SHAPE through MeshManager::Load(file_name)

              // Find out if these are useful and if so how to mesh them:
              // physics::Base::POLYLINE_SHAPE
              // physics::Base::HEIGHTMAP_SHAPE
              // physics::Base::MAP_SHAPE
              // physics::Base::MULTIRAY_SHAPE
              // physics::Base::RAY_SHAPE
            }
          } else {
            gzwarn << "Geometry type not available" << std::endl;
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

  std::vector<std::string> mesh_types_names_ =
      {"POINTS", "LINES", "LINESTRIPS", "TRIANGLES", "TRIFANS", "TRISTRIPS"};
};
}  // namespace gazebo

#endif //VOXBLOX_GROUND_TRUTH_GAZEBO_PLUGIN_H
