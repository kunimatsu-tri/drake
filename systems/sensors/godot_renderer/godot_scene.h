#pragma once

#include <string>
#include <map>
#include <utility>

#include <Eigen/Dense>

#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/camera.h"
#include "scene/resources/primitive_meshes.h"

namespace godotvis {

class GodotScene {
  SceneTree* tree_ = NULL;
  Spatial* scene_root_ = NULL; //!< This will be freed by scene tree. Keep the pointer to add instances.
  Camera* camera_ = NULL; //!< This will be freed by scene tree. Keep the pointer to update.
  Environment* env = NULL;
  MeshInstance* mesh_instance_ = NULL;
  Ref<ShaderMaterial> shader_material_;
  //SpatialMaterial* material = NULL;
  CubeMesh* cube_ = nullptr;
  SphereMesh* sphere_ = nullptr;
  CylinderMesh* cylinder_ = nullptr;
  PlaneMesh* plane_ = nullptr;

  /// List of materials for a mesh, one for each surface
  using MaterialList = std::vector<Ref<SpatialMaterial>>;
  /// Mapping from instance id to its material list
  std::map<int, MaterialList> instance_materials_;

  struct MeshMaterialsPair {
    Ref<Mesh> mesh;
    MaterialList materials;
  };

public:
  GodotScene() {}

  void Initialize();
  void SetupEnvironment(const std::string& env_filename);
  void SetBackgroundColor(float r, float g, float b);

  void ImportGltf(const std::string& file_name);

  /// Add a camera to the scene. Only support one camera for now.
  /// TODO: handle distortion
  void AddCamera(double fov_y, double z_near, double z_far);

  void SetCameraPose(const Eigen::Isometry3d& X_WI);

  /// Set the size for the viewport, which also determines the camera
  /// image size. This can be set multiple times, however, the first
  /// set of values are the max size (for some unknown reason)
  /// Note that this is independent of the renderer's window size.
  void set_viewport_size(int width, int height);

  std::pair<int, int> get_viewport_size() const;
  double get_camera_fov_y() const;

  Ref<Image> Capture();
  void ApplyDepthShader();
  void ApplyMaterialShader();
  void Finish();
  int AddMeshInstance(const std::string& filename);
  int AddCubeInstance(double x_length, double y_length, double z_length);
  int AddSphereInstance(double radius);
  int AddCylinderInstance(double radius, double height);
  int AddPlaneInstance(double x_size, double y_size);

  void SetInstanceTranslation(int id, const Eigen::Vector3d& t_WI);
  void SetInstanceRotation(int id, const Eigen::Isometry3d& t_WI);

  void SetInstancePose(int id, const Eigen::Isometry3d& X_WI);

  void SetInstancePose(Spatial* instance, const Eigen::Isometry3d& X_WI);

  void SetInstanceScale(int id, double sx, double sy, double sz);
  void SetInstanceColor(int id, float r, float g, float b, float alpha = 1.0f);
  void FlushTransformNotifications();

  void AddOmniLight(double x, double y, double z);
  void AddDirectionalLight(double d_x, double d_y, double d_z);

private:

  static Ref<SpatialMaterial> MakeSimplePlastic(double r, double g, double b);
  void InitDepthShader();
  Spatial* get_spatial_instance(int id);
  MeshMaterialsPair LoadMesh(const std::string& filename);
  int AddInstance(const MeshMaterialsPair& mesh_materials);
  void SetInstanceLocalTransform(int id, const Eigen::Isometry3d& X_WI);
};

/// Utility functon to convert Eigen's transform to Godot's transform
Transform ConvertToGodotTransform(const Eigen::Isometry3d& transform);

} // namespace godotvis

