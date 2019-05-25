#include "voxblox_ground_truth/user_interfaces/gazebo_plugin.h"
#include <string>
#include "voxblox_ground_truth/sdf_creator.h"

namespace gazebo {
GZ_REGISTER_WORLD_PLUGIN(VoxbloxGroundTruthPlugin)

VoxbloxGroundTruthPlugin::VoxbloxGroundTruthPlugin()
    : WorldPlugin(), nh_private_("~"), sdf_visualizer_(&nh_private_) {
}

void VoxbloxGroundTruthPlugin::Load(physics::WorldPtr world,
                                    sdf::ElementPtr _sdf) {
  world_ = world;

  // Advertise the TSDF generation service
  std::string service_name = "save_voxblox_ground_truth_to_file";
  LOG(INFO) << "Advertising service: " << service_name;
  srv_ = nh_private_.advertiseService(
      service_name, &VoxbloxGroundTruthPlugin::serviceCallback, this);
}

bool VoxbloxGroundTruthPlugin::serviceCallback(
    voxblox_msgs::FilePath::Request &request,
    voxblox_msgs::FilePath::Response &response) {
  // Read the voxel size from ROS params
  CHECK(nh_private_.getParam("/voxblox_ground_truth/voxel_size", voxel_size_))
      << "ROS param /voxblox_ground_truth/voxel_size must be set.";

  // Instantiate a Gazebo mesh manager
  common::MeshManager *mesh_manager = common::MeshManager::Instance();
  CHECK_NOTNULL(mesh_manager);

  // Instantiate the ground truth SDF creator
  voxblox::TsdfMap::Config map_config;
  map_config.tsdf_voxel_size = voxel_size_;
  voxblox_ground_truth::SdfCreator sdf_creator(map_config);

  // Iterate over all collision geometries
  for (const physics::ModelPtr &model : world_->GetModels()) {
    for (const physics::LinkPtr &link : model->GetLinks()) {
      for (const physics::CollisionPtr &collision : link->GetCollisions()) {
        LOG(INFO) << "Processing '" << collision->GetScopedName(true) << "'";

        // Convert the geometry shape to a proto message
        // NOTE: This is done such that we can read the shape properties through
        //       methods from msgs::Geometry, whose names are human friendlier
        msgs::Geometry geometry_msg;
        collision->GetShape()->FillMsg(geometry_msg);

        LOG(INFO) << "------------------ SDF ------------------";
        LOG(INFO) << collision->GetShape()->GetSDF()->ToString("");
        LOG(INFO) << "-----------------------------------------";

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
                  LOG(ERROR) << "Encountered multiple sub meshes"
                             << "\nSkipping this mesh.";
                  continue;
                }

                // Create a copy of the submesh s.t. it can be manipulated
                common::SubMesh submesh(mesh_ptr->GetSubMesh(0));

                // Make sure we're dealing with a triangle mesh
                if (submesh.GetPrimitiveType() != common::SubMesh::TRIANGLES) {
                  std::string mesh_type_str =
                      mesh_type_names_[submesh.GetPrimitiveType()];
                  LOG(ERROR) << "Encountered a mesh with type " << mesh_type_str
                             << ". Currently, "
                             << "only triangular meshes are supported."
                             << "\nSkipping this mesh.";
                  continue;
                }

                // Find the geometry size
                // NOTE: There is no need to scale the geometry, since
                //       Gazebo already returns it at the appropriate scale
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
                  LOG(ERROR) << "Could not get geometry size";
                  continue;
                }

                // Scale the mesh and transform it into world frame
                const ignition::math::Pose3d transform =
                    collision->GetWorldPose().Ign();
                for (unsigned int vertex_i = 0;
                     vertex_i < submesh.GetVertexCount(); vertex_i++) {
                  // Create a copy of the vertex s.t. it can be manipulated
                  ignition::math::Vector3d new_vertex =
                      submesh.Vertex(vertex_i);

                  // Scale and transform it into world frame
                  new_vertex *= geometry_size;
                  new_vertex = transform.Rot() * new_vertex;
                  new_vertex += transform.Pos();

                  // Add the vertex to the mesh
                  submesh.SetVertex(vertex_i, new_vertex);
                }

                // Integrate the mesh faces (triangles) into the SDF
                unsigned int num_faces = submesh.GetIndexCount() / 3;
                LOG(INFO) << "Integrating " << num_faces << " faces";
                for (unsigned int triangle_i = 0; triangle_i < num_faces;
                     triangle_i++) {
                  // Get the indices of the vertices
                  const unsigned int index_a = submesh.GetIndex(triangle_i * 3);
                  const unsigned int index_b =
                      submesh.GetIndex(triangle_i * 3 + 1);
                  const unsigned int index_c =
                      submesh.GetIndex(triangle_i * 3 + 2);

                  // Get the coordinates of the vertices
                  TriangularFaceVertexCoordinates triangle_vertices;
                  triangle_vertices.vertex_a = {
                      static_cast<float>(submesh.Vertex(index_a).X()),
                      static_cast<float>(submesh.Vertex(index_a).Y()),
                      static_cast<float>(submesh.Vertex(index_a).Z())};
                  triangle_vertices.vertex_b = {
                      static_cast<float>(submesh.Vertex(index_b).X()),
                      static_cast<float>(submesh.Vertex(index_b).Y()),
                      static_cast<float>(submesh.Vertex(index_b).Z())};
                  triangle_vertices.vertex_c = {
                      static_cast<float>(submesh.Vertex(index_c).X()),
                      static_cast<float>(submesh.Vertex(index_c).Y()),
                      static_cast<float>(submesh.Vertex(index_c).Z())};

                  // Integrate the triangle into the mesh
                  sdf_creator.integrateTriangle(triangle_vertices);
                }
              } else {
                LOG(WARNING) << "Could not get pointer to mesh '" << mesh_name
                             << "'";
                return false;
              }
            } else {
              LOG(WARNING) << "Could not get pointer to MeshManager";
              return false;
            }
          } else {
            // TODO(victorr): Add support for remaining Mesh shapes, namely
            //                - physics::Base::POLYLINE_SHAPE
            //                - physics::Base::HEIGHTMAP_SHAPE
            //                - physics::Base::MAP_SHAPE
            //                - physics::Base::MULTIRAY_SHAPE
            //                - physics::Base::RAY_SHAPE
            LOG(WARNING) << "Not yet able to process shapes of type: "
                         << geometry_type_str;
            return false;
          }
        } else {
          LOG(WARNING) << "Geometry type not available";
          return false;
        }
      }
    }
  }

  // Visualize the TSDF and intersection count layers
  bool publish_debug_visuals = true;
  nh_private_.param("publish_visuals", publish_debug_visuals,
                    publish_debug_visuals);
  if (publish_debug_visuals) {
    sdf_visualizer_.publishTsdfVisuals(
        sdf_creator.getTsdfMap().getTsdfLayer());
    sdf_visualizer_.publishIntersectionVisuals(
        sdf_creator.getIntersectionLayer());
  }

  // Save the TSDF to a file
  LOG(INFO) << "Saving TSDF to file: " << request.file_path;
  sdf_creator.getTsdfMap().getTsdfLayer().saveToFile(request.file_path, true);

  return true;
}
}  // namespace gazebo
