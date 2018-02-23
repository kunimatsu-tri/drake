#include "drake/systems/sensors/godot_renderer/godot_scene.h"

#include <iostream>
#include <stdexcept>

#include <core/io/image_loader.h>
#include <editor/import/editor_scene_importer_gltf.h>
#include <servers/visual/visual_server_global.h>

#include <editor/editor_node.h>

// stub-out Godot functions that we don't link
void EditorNode::progress_add_task(const String &p_task, const String &p_label, int p_steps, bool p_can_cancel ) {}
void EditorNode::progress_end_task(const String &p_task) {}

namespace godotvis {

void GodotScene::Initialize() {
  // Construct and initialize a SceneTree
  // This creates a default Viewport as the root
  // and a World attached to that Viewport
  // The World creates and contains a VisualServer's scenario
  // We need to attach an environment to the World and other
  // visual instance (camera, meshes etc) to the tree_.
  tree_ = memnew(SceneTree);
  tree_->init();

  // Dummy Spatial as the top root of the scene
  scene_root_ = memnew(Spatial);
  tree_->add_current_scene(scene_root_); // need to do this here so all
                                         // subsequent children knows about the
                                         // tree_, espcially the camera

  InitDepthShader();
}

void GodotScene::SetupEnvironment(const std::string& env_filename) {
  // Load skybox resource
  String skyfilename{env_filename.c_str()};
#if 1
  Ref<Image> image;
  image.instance();
  Error err = ImageLoader::load_image(skyfilename, image);
  if (err != OK) {
    // TODO(SeanCurtis-TRI): Create default environment and log error.
    std::cerr << "Unable to load: " << env_filename << "\n";
    return;
  }

  Ref<ImageTexture> sky_texture;
  sky_texture.instance();
  sky_texture->create_from_image(image);
#else
  // TODO(SeanCurtis-TRI): Enable this when our image loader is tied into
  // ResourceLoader.
  Ref<Texture> sky_texture = ResourceLoader::load(skyfilename);
#endif
  // TODO: This will be freed by Environment. Also, what's the correct way to
  // create a Ref<Sky>?
  PanoramaSky *sky = memnew(PanoramaSky);
  sky->set_panorama(sky_texture);
  sky->set_radiance_size(Sky::RADIANCE_SIZE_64);

  //// Set Environment
  env = memnew(Environment);
  env->set_background(Environment::BG_SKY);
  env->set_sky(sky);
  env->set_bg_energy(5.0);
  tree_->get_root()->get_world()->set_environment(env);
  // Add lights
  Light *light = memnew(OmniLight());
  scene_root_->add_child(light);
  light->set_color(Color(1.0, 1.0, 1.0));
  light->set_param(Light::PARAM_ENERGY, 2.0);
  light->set_param(Light::PARAM_RANGE, 50.0);
  light->set_transform(Transform(Basis(), Vector3(1.0, 5., 10.)));
}

void GodotScene::SetBackgroundColor(float r, float g, float b) {
  //TODO(duy): alpha doesn't work!!! Godot switches it to 1.0 somewhere?
  env->set_bg_color(Color{r, g, b, 1.0f});
}

void GodotScene::ImportGltf(const std::string& file_name) {
  EditorSceneImporterGLTF importer;
  // The importer interface has many arguments that we don't require:
  // flags: `p_flags` (unused by gltf parsing) and `p_bake_fps` (we don't
  //   support animation.
  // The error handling (list of missing dependencies and error code is *not*
  // currently implemented in the gltf parsing.
  List<String> missing_dependencies;
  Error error{OK};
  Node* node = importer.import_scene(file_name.c_str(), 0, 0,
                                     &missing_dependencies, &error);
  if (node) {
    scene_root_->add_child(node);
  } else {
    // TODO(SeanCurtis-TRI): Make use of the error message when the importer
    // likewise makes use of it.
    throw std::runtime_error("Unable to load the file: " + file_name);
  }
  SpatialMaterial::flush_changes();
}

/// Add a camera to the scene. Only support one camera for now.
/// TODO: handle distortion
void GodotScene::AddCamera(double fov_y, double z_near, double z_far) {
  if (camera_)
    throw std::runtime_error("Cannot add: a camera already exists!");
  camera_ = memnew(Camera);
  scene_root_->add_child(camera_);
  camera_->set_perspective(fov_y, z_near, z_far);
}

Ref<Image> GodotScene::Capture() {
  return tree_->get_root()->get_texture()->get_data();
}

void GodotScene::ApplyDepthShader() {
  for (const auto pair : instance_materials_) {
    int id = pair.first;
    MeshInstance *instance =
        Object::cast_to<MeshInstance>(scene_root_->get_child(id));
    Ref<Mesh> mesh = instance->get_mesh();
    for (int i = 0; i < mesh->get_surface_count(); ++i)
      instance->set_surface_material(i, shader_material_);
  }
  SpatialMaterial::flush_changes();
  env->set_background(Environment::BG_CLEAR_COLOR);
}

void GodotScene::ApplyMaterialShader() {
  for (const auto pair : instance_materials_) {
    int id = pair.first;
    const MaterialList& materials = pair.second;
    MeshInstance *instance =
        Object::cast_to<MeshInstance>(scene_root_->get_child(id));
    for (size_t i = 0; i < materials.size(); ++i)
      instance->set_surface_material(i, materials[i]);
  }
  SpatialMaterial::flush_changes();
  env->set_background(Environment::BG_SKY);
}

void GodotScene::Finish() {
  VSG::storage->update_dirty_resources();
  shader_material_.unref();
  if (tree_) {
    // This will free all the resources/instances that have been new-ed
    // and added to the tree
    tree_->finish();
    memdelete(tree_);
  }
}

int GodotScene::AddInstance(const MeshMaterialsPair& mesh_materials) {
  MeshInstance *instance = memnew(MeshInstance);
  scene_root_->add_child(instance); // Must add before doing any settings
  instance->set_mesh(mesh_materials.mesh);
  instance->set_notify_local_transform(true);
  instance->set_notify_transform(true);
  int id = instance->get_position_in_parent();
  instance_materials_[id] = mesh_materials.materials;
  for (size_t i = 0; i < mesh_materials.materials.size(); ++i)
    instance->set_surface_material(i, mesh_materials.materials[i]);
  return id;
}

int GodotScene::AddMeshInstance(const std::string &filename) {
  return AddInstance(LoadMesh(filename));
}

int GodotScene::AddCubeInstance(double x_length, double y_length,
                                double z_length) {
  if (!cube_) {
    cube_ = memnew(CubeMesh);
    cube_->set_size(Vector3(1.0, 1.0, 1.0));
  }
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
  material->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
  material->set_albedo(Color(1.0, 0.0, 0.0));
  material->set_script_instance(nullptr);
  SpatialMaterial::flush_changes();
  int id = AddInstance(MeshMaterialsPair{cube_, {material}});
  SetInstanceScale(id, x_length, y_length, z_length);
  return id;
}

int GodotScene::AddSphereInstance(double radius) {
  if (!sphere_) {
    sphere_ = memnew(SphereMesh);
    sphere_->set_radius(0.5);
    sphere_->set_height(1.0);
  }
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
  material->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
  material->set_albedo(Color(0.0, 1.0, 0.0));
  material->set_script_instance(nullptr);
  SpatialMaterial::flush_changes();
  int id = AddInstance(MeshMaterialsPair{sphere_, {material}});
  SetInstanceScale(id, radius*2.0, radius*2.0, radius*2.0);
  return id;
}

int GodotScene::AddCylinderInstance(double radius, double height) {
  if (!cylinder_) {
    cylinder_ = memnew(CylinderMesh);
    cylinder_->set_top_radius(0.5);
    cylinder_->set_bottom_radius(0.5);
    cylinder_->set_height(1.0);
  }
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
  material->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
  material->set_albedo(Color(0.0, 0.0, 1.0));
  SpatialMaterial::flush_changes();
  int id = AddInstance(MeshMaterialsPair{cylinder_, {material}});
  // this call rotates the mesh into Drake's convention
  SetInstancePose(id, Eigen::Isometry3d::Identity());
  SetInstanceScale(id, radius*2.0, height, radius*2.0);
  return id;
}

int GodotScene::AddPlaneInstance(double x_size, double y_size) {
  if (!plane_) {
    plane_ = memnew(PlaneMesh);
    plane_->set_size(Size2(1.0, 1.0));
  }
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
  material->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
  material->set_albedo(Color(1.0, 1.0, 1.0));
  SpatialMaterial::flush_changes();
  int id = AddInstance(MeshMaterialsPair{plane_, {material}});
  // this call rotates the mesh into Drake's convention
  SetInstancePose(id, Eigen::Isometry3d::Identity());
  SetInstanceScale(id, x_size, 1.0, y_size);
  return id;
}

void GodotScene::InitDepthShader() {
  shader_material_.instance();
  Shader *shader = memnew(Shader);
  String code = "shader_type spatial; \nrender_mode unshaded;\n";
  code += "void fragment() {\n";
  code += "\tfloat near = 2.0, far = 3.0;\n";
  code += "\tfloat dis = -VERTEX.z; // TODO can easily switch to real distance "
          "instead of just z\n";
  code += "\tfloat depth_color = (dis-near)/(far-near)*1.0;\n";
  code += "\tvec3 viewing_ray = normalize(VERTEX);\n";
  code += "\tif (abs(dot(NORMAL, viewing_ray)) < 0.3) {\n";
  code += "\t\t	depth_color = 0.0;\n";
  code += "\t}\n";
  code += "\tALBEDO = vec3(depth_color, depth_color, depth_color);\n";
  code += "\tALPHA = 1.0;\n";
  code += "}\n";
  shader->set_code(code);
  shader_material_->set_shader(shader);
}

Spatial *GodotScene::get_spatial_instance(int id) {
  Node *instance = scene_root_->get_child(id);
  return Object::cast_to<Spatial>(instance);
}

GodotScene::MeshMaterialsPair GodotScene::LoadMesh(const std::string &filename) {
  // TODO: properly load a glTF file here!
  // Load a mesh
  // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(),
  // which initializes VAO
  // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <--
  // ArrayMesh::_setv() <-- res->set() <--
  // ResourceInteractiveLoaderBinary::poll()
  Ref<ArrayMesh> mesh = ResourceLoader::load(String(filename.c_str()));

  // Load its material
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
  material->set_albedo(Color(1.0, 1.0, 1.0, 1.0));

  // TODO: remove this hack
  String path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";
  Ref<Texture> texture = ResourceLoader::load(path + "aluminium_albedo.png");
  material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
  material->set_metallic(0.59);
  material->set_specular(0.5);
  material->set_roughness(0.4);
  Ref<Texture> normal_tex = ResourceLoader::load(path + "aluminium_normal.png");
  material->set_feature(SpatialMaterial::FEATURE_NORMAL_MAPPING, true);
  material->set_texture(SpatialMaterial::TEXTURE_NORMAL, normal_tex);
  material->set_normal_scale(0.21);
  material->set_feature(SpatialMaterial::FEATURE_ANISOTROPY, true);
  material->set_anisotropy(0.99);
  // IMPORTANT: This calls to SpatialMaterial::_update_shader(). Without this
  // materials wont' work
  // Godot sets up so that this is called in SceneTree::call_idle_call_backs()
  // Not sure if we need to do this as an idle callback...
  SpatialMaterial::flush_changes();
  mesh->surface_set_material(0, material);
  mesh->surface_set_material(1, material);
  mesh->surface_set_material(2, material);
  mesh->surface_set_material(3, material);

  MaterialList materials;
  for(int i = 0; i<4; ++i)
    materials.push_back(material);

  return GodotScene::MeshMaterialsPair{mesh, materials};
}

void GodotScene::set_viewport_size(int width, int height) {
  tree_->get_root()->set_size(Size2(width, height));
}

std::pair<int,int> GodotScene::get_viewport_size() const {
  Size2 size = tree_->get_root()->get_size();
  return std::make_pair(static_cast<int>(size[0]), static_cast<int>(size[1]));
}

double GodotScene::get_camera_fov_y() const {
  return camera_->get_fov();
}

Transform ConvertToGodotTransform(const Eigen::Isometry3d& transform) {
  const auto& R = transform.rotation();
  const auto& t = transform.translation();
  return Transform(Basis(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2),
                         R(2, 0), R(2, 1), R(2, 2)),
                   Vector3(t(0), t(1), t(2)));
}

void GodotScene::SetInstancePose(Spatial* instance,
                                 const Eigen::Isometry3d& X_WI) {
  const auto& t = X_WI.translation();
  instance->set_translation(Vector3(t[0], t[1], t[2]));
  const auto& R = X_WI.rotation();
  // clang-format off
  Basis basis(R(0, 0), R(0, 1), R(0, 2),
              R(1, 0), R(1, 1), R(1, 2),
              R(2, 0), R(2, 1), R(2, 2));
  // clang-format on
  // TODO(duy): Add comment here about Godot's plane coordinate frame
  // and the differences of set_transform vs set_rotation
  Mesh* mesh = Object::cast_to<MeshInstance>(instance)->get_mesh().ptr();
  if (mesh->get_class_name() == "CylinderMesh" ||
      mesh->get_class_name() == "PlaneMesh") {
    // clang-format off
    basis = Basis(1., 0., 0.,
                  0., 0., -1.,
                  0., 1., 0.) * basis;
    // clang-format on
  }
  instance->set_rotation(basis.get_rotation());
}

void GodotScene::SetInstancePose(int id, const Eigen::Isometry3d& X_WI) {
  Spatial *instance = get_spatial_instance(id);
  SetInstancePose(instance, X_WI);
}

void GodotScene::SetCameraPose(const Eigen::Isometry3d& X_WC) {
  std::cout << "Set camera pose: " << X_WC.translation().transpose() << std::endl;
  std::cout << X_WC.rotation() << std::endl;
  //camera_->set_transform(ConvertToGodotTransform(X_WC));
  camera_->set_transform(ConvertToGodotTransform(Eigen::Isometry3d::Identity()));
  const auto& t = X_WC.translation();
  camera_->set_translation(Vector3(t[0], t[1], t[2]));
  const auto& R = X_WC.rotation();
  // clang-format off
  Basis basis(R(0, 0), R(0, 1), R(0, 2),
              R(1, 0), R(1, 1), R(1, 2),
              R(2, 0), R(2, 1), R(2, 2));
  // clang-format on
  camera_->set_rotation(basis.get_rotation());
}

void GodotScene::SetInstanceScale(int id, double sx, double sy, double sz) {
  Spatial *instance = get_spatial_instance(id);
  instance->set_scale(Vector3(sx, sy, sz));
}

void GodotScene::SetInstanceColor(int id, float r, float g, float b,
                                  float alpha) {
  for(auto& material: instance_materials_[id]) {
    material->set_albedo(Color(r, g, b, alpha));
  }
  SpatialMaterial::flush_changes();
}

void GodotScene::SetInstanceLocalTransform(int id,
                                 const Eigen::Isometry3d& X_WI) {
  Spatial *instance = get_spatial_instance(id);
  instance->set_transform(ConvertToGodotTransform(X_WI));
}

void GodotScene::FlushTransformNotifications() {
  tree_->flush_transform_notifications();
}

} // namespace godotvis

