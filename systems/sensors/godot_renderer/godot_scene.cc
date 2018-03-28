#include "drake/systems/sensors/godot_renderer/godot_scene.h"

#include <iostream>
#include <stdexcept>

#include <core/io/image_loader.h>
#include <editor/import/editor_scene_importer_gltf.h>
#include <servers/visual/visual_server_global.h>

#include <editor/editor_node.h>
#include <queue>

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
  tree_->get_root()->set_msaa(Viewport::MSAA_16X);
  tree_->get_root()->set_shadow_atlas_size(4096);

  // Dummy Spatial as the top root of the scene
  scene_root_ = memnew(Spatial);
  tree_->add_current_scene(scene_root_); // need to do this here so all
                                         // subsequent children knows about the
                                         // tree_, especially the camera

  InitDepthShader();
}

DirectionalLight* GodotScene::AddDirectionalLight(double d_x, double d_y, double d_z) {

  // Directional lights are initialized pointing down the -z axis.
  DirectionalLight* d_light = memnew(DirectionalLight);
  d_light->set_color(Color(1, 1, 1));
  Basis basis;
  basis.from_z(Vector3(d_x, d_y, d_z));
  d_light->set_transform(Transform(basis));
  // TODO(SeanCurtis-TRI): Make this a parameter.
//  d_light->set_shadow(true);
  // The shadow max distance must be >= the distance between camera and surface
  // receiving the shadow.
  // TODO(SeanCurtis-TRI): Parameterize this value based on "scene size".
  d_light->set_param(Light::PARAM_SHADOW_MAX_DISTANCE, 1);
  scene_root_->add_child(d_light);
  return d_light;
}

SpotLight* GodotScene::AddSpotLight() {
  // NOTE: By default, spotlights point in the -z direction.
  SpotLight* light = memnew(SpotLight);
  scene_root_->add_child(light);

  // Spot light parameters
  light->set_param(Light::PARAM_RANGE, 3.0);
  light->set_param(Light::PARAM_ATTENUATION, 0.05);
  light->set_param(SpotLight::PARAM_SPOT_ANGLE, 45);
  light->set_param(SpotLight::PARAM_SPOT_ATTENUATION, 30);

  // Light parameters
  light->set_color(Color(1.0, 1.0, 1.0));
  light->set_param(Light::PARAM_ENERGY, 1);
  light->set_param(Light::PARAM_INDIRECT_ENERGY, 1);
  light->set_negative(false);
  light->set_param(Light::PARAM_SPECULAR, 0.5);
  light->set_bake_mode(Light::BAKE_INDIRECT);

  // Shadow parameters
  light->set_shadow(true);
  light->set_param(Light::PARAM_SHADOW_BIAS, 0.01);

  return light;
}

OmniLight* GodotScene::AddOmniLight(double x, double y, double z) {
  OmniLight* light = memnew(OmniLight);
  scene_root_->add_child(light);
  // Omni light parameters
  light->set_param(Light::PARAM_RANGE, 3.0);
  light->set_param(Light::PARAM_ATTENUATION, 1.0);
  light->set_shadow_mode(OmniLight::SHADOW_CUBE);
  light->set_shadow_detail(OmniLight::SHADOW_DETAIL_HORIZONTAL);
  // Light parameters
  light->set_color(Color(1.0, 1.0, 1.0));
  light->set_param(Light::PARAM_ENERGY, 1);
  light->set_param(Light::PARAM_INDIRECT_ENERGY, 1);
  light->set_negative(false);
  light->set_param(Light::PARAM_SPECULAR, 0.5);
  light->set_bake_mode(Light::BAKE_INDIRECT);
  // Shadow parameters
  light->set_shadow(true);
  light->set_param(Light::PARAM_SHADOW_BIAS, 0.01);

  light->set_transform(Transform(Basis(), Vector3(x, y, z)));
  return light;
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
  PanoramaSky* sky = memnew(PanoramaSky);
  sky->set_panorama(sky_texture);
  sky->set_radiance_size(Sky::RADIANCE_SIZE_64);

  //// Set Environment
  env = memnew(Environment);
  env->set_background(Environment::BG_SKY);
  env->set_sky(sky);
  env->set_bg_energy(5.0);
  tree_->get_root()->get_world()->set_environment(env);

  SpotLight* light = AddSpotLight();
  light->set_param(Light::PARAM_RANGE, 2);
  light->set_transform(Transform(Basis(), Vector3(0, 0, 1.5)));
  light->set_param(SpotLight::PARAM_SPOT_ANGLE, 30);
}

void GodotScene::SetBackgroundColor(float r, float g, float b) {
  //TODO(duy): alpha doesn't work!!! Godot switches it to 1.0 somewhere?
  env->set_bg_color(Color{r, g, b, 1.0f});
}

// Find the first node in the tree rooted at `node` (via a breadth-first search)
// that is a MeshInstance. Throw an exception if none can be found. This will
// ignore multiple MeshInstance nodes; remove it from the tree and return it.
MeshInstance* FindMeshInstance(Node* node) {
  std::queue<std::pair<Node*, Node*>> q;
  q.push(std::make_pair(nullptr, node));
  while (!q.empty()) {
    std::pair<Node*, Node*> pair = q.front();
    Node* p = pair.first;
    Node* n = pair.second;
    q.pop();
    MeshInstance* instance = Object::cast_to<MeshInstance>(n);
    if (instance != nullptr) {
      if (p != nullptr) p->remove_child(n);
      return instance;
    }

    for (int c = 0; c < n->get_child_count(); ++c) {
      q.push(std::make_pair(n, n->get_child(c)));
    }
  }
  throw std::runtime_error("Couldn't find MeshInstance in the given Node");
}

int GodotScene::ImportGltf(const std::string& file_name,
                           Ref<SpatialMaterial> label) {
  // TODO(SeanCurtis-TRI): Handle label color!
  EditorSceneImporterGLTF importer;
  // The importer interface has many arguments that we don't require:
  // flags: `p_flags` (unused by gltf parsing) and `p_bake_fps` (we don't
  //   support animation.
  // The error handling (list of missing dependencies and error code) is *not*
  // currently implemented in the gltf parsing.
  List<String> missing_dependencies;
  Error error{OK};
  Node* node = importer.import_scene(file_name.c_str(), 0, 0,
                                     &missing_dependencies, &error);
  int id;
  if (node) {
    // Strip out the *single* allowable MeshInstance
    MeshInstance* mesh_instance = FindMeshInstance(node);
    scene_root_->add_child(mesh_instance);
    id = mesh_instance->get_position_in_parent();
    label_materials_[id] = label;
    MaterialList materials;
    Ref<Mesh> mesh = mesh_instance->get_mesh();
    for (int i = 0; i < mesh->get_surface_count(); ++i)
      materials.push_back(mesh_instance->get_surface_material(i));
    instance_materials_[id] = materials;
    if (mesh_instance != node) memdelete(node);
    // TODO(SeanCurtis-TRI): Walk the tree and store all materials. Would it be
    // better to have two versions of this and swap them?
  } else {
    // TODO(SeanCurtis-TRI): Make use of the error message when the importer
    // likewise makes use of it.
    throw std::runtime_error("Unable to load the file: " + file_name);
  }
  SpatialMaterial::flush_changes();

  return id;
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

MeshInstance* GodotScene::get_mesh_instance(int id) {
  return Object::cast_to<MeshInstance>(scene_root_->get_child(id));
}

void GodotScene::ApplyLabelShader() {
  tree_->get_root()->set_msaa(Viewport::MSAA_DISABLED);
  for (const auto pair : label_materials_) {
    int id = pair.first;
    MeshInstance *instance =
        Object::cast_to<MeshInstance>(scene_root_->get_child(id));
    Ref<SpatialMaterial> material = label_materials_[id];
    Ref<Mesh> mesh = instance->get_mesh();
    for (int i = 0; i < mesh->get_surface_count(); ++i)
      instance->set_surface_material(i, material);
  }
  SpatialMaterial::flush_changes();
  env->set_background(Environment::BG_CLEAR_COLOR);
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
  tree_->get_root()->set_msaa(Viewport::MSAA_16X);
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

int GodotScene::AddInstance(const MeshMaterialsPair& mesh_materials,
                            Ref<SpatialMaterial> label) {
  MeshInstance *instance = memnew(MeshInstance);
  scene_root_->add_child(instance); // Must add before doing any settings
  instance->set_mesh(mesh_materials.mesh);
  instance->set_notify_local_transform(true);
  instance->set_notify_transform(true);
  int id = instance->get_position_in_parent();
  label_materials_[id] = label;
  instance_materials_[id] = mesh_materials.materials;
  for (size_t i = 0; i < mesh_materials.materials.size(); ++i)
    instance->set_surface_material(i, mesh_materials.materials[i]);
  return id;
}

int GodotScene::AddMeshInstance(const std::string &filename, const Color& color,
                                const Color& label_color) {
  std::string extension = filename.substr(filename.size() - 4);
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  Ref<SpatialMaterial> label = MakeLabelMaterial(label_color);
  if (extension == "mesh") {
    return AddInstance(LoadMesh(filename, color), label);
  } else if (extension == "gltf") {
    return ImportGltf(filename, label);
  } else {
    throw std::logic_error(
        "Add mesh instance should only be invoked with mesh "
        "or gltf files; invoked on: " +
        filename);
  }
}

int GodotScene::AddCubeInstance(double x_length, double y_length,
                                double z_length, const Color& color,
                                const Color& label_color) {
  if (!cube_) {
    cube_ = memnew(CubeMesh);
    cube_->set_size(Vector3(1.0, 1.0, 1.0));
  }
  Ref<SpatialMaterial> material = MakeSimplePlastic(color);
  Ref<SpatialMaterial> label = MakeLabelMaterial(label_color);
  int id = AddInstance(MeshMaterialsPair{cube_, {material}}, label);
  SetInstanceScale(id, x_length, y_length, z_length);
  return id;
}

int GodotScene::AddSphereInstance(double radius, const Color& color,
                                  const Color& label_color) {
  if (!sphere_) {
    // Unit sphere - radius = 1. Godot defines "spheres" with a radius and
    // height. The radius is the radius of the equator the height is the total
    // height of the sphere. So, a sphere has radius r and height 2r.
    sphere_ = memnew(SphereMesh);
    sphere_->set_radius(1.0);
    sphere_->set_height(2.0);
  }
  Ref<SpatialMaterial> material = MakeSimplePlastic(color);
  Ref<SpatialMaterial> label = MakeLabelMaterial(label_color);
  int id = AddInstance(MeshMaterialsPair{sphere_, {material}}, label);
  SetInstanceScale(id, radius, radius, radius);
  return id;
}

int GodotScene::AddCylinderInstance(double radius, double height,
                                    const Color& color,
                                    const Color& label_color) {
  if (!cylinder_) {
    cylinder_ = memnew(CylinderMesh);
    cylinder_->set_top_radius(0.5);
    cylinder_->set_bottom_radius(0.5);
    cylinder_->set_height(1.0);
  }
  Ref<SpatialMaterial> material = MakeSimplePlastic(color);
  Ref<SpatialMaterial> label = MakeLabelMaterial(label_color);
  int id = AddInstance(MeshMaterialsPair{cylinder_, {material}}, label);
  // this call rotates the mesh into Drake's convention
  SetInstancePose(id, Eigen::Isometry3d::Identity());
  SetInstanceScale(id, radius*2.0, height, radius*2.0);
  return id;
}

int GodotScene::AddPlaneInstance(double x_size, double y_size,
                                 const Color& color,
                                 const Color& label_color) {
  if (!plane_) {
    plane_ = memnew(PlaneMesh);
    plane_->set_size(Size2(1.0, 1.0));
  }

  Ref<SpatialMaterial> material = MakeSimplePlastic(color);
  Ref<SpatialMaterial> label = MakeLabelMaterial(label_color);
  int id = AddInstance(MeshMaterialsPair{plane_, {material}}, label);
  // TODO(SeanCurtis-TRI): Inject a parent transform that takes the pose and
  // allows the plane's transform to rotate it on the x-y plane (as opposed to
  // the godot x-z default plane).
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

GodotScene::MeshMaterialsPair GodotScene::LoadMesh(const std::string &filename,
                                                   const Color& color) {
  // TODO: properly load a glTF file here!
  // Load a mesh
  // This calls all the way down to RasterizerStorageGLES3::mesh_add_surface(),
  // which initializes VAO
  // RasterizerStorageGLES3::mesh_add_surface <-- ArrayMesh::add_surface <--
  // ArrayMesh::_setv() <-- res->set() <--
  // ResourceInteractiveLoaderBinary::poll()
  Ref<ArrayMesh> mesh = ResourceLoader::load(String(filename.c_str()));

  // Load its material
  Ref<SpatialMaterial> material = MakeSimplePlastic(color);
  material->set_metallic(1.);
  material->set_roughness(0.2);
  // TODO: remove this hack
//  String path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";
//  Ref<Texture> texture = ResourceLoader::load(path + "aluminium_albedo.png");
//  material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
//  Ref<Texture> normal_tex = ResourceLoader::load(path + "aluminium_normal.png");
//  material->set_feature(SpatialMaterial::FEATURE_NORMAL_MAPPING, true);
//  material->set_texture(SpatialMaterial::TEXTURE_NORMAL, normal_tex);
//  material->set_normal_scale(0.21);
//  material->set_feature(SpatialMaterial::FEATURE_ANISOTROPY, true);
//  material->set_anisotropy(0.99);

  // IMPORTANT: This calls to SpatialMaterial::_update_shader(). Without this
  // materials wont' work
  // Godot sets up so that this is called in SceneTree::call_idle_call_backs()
  // Not sure if we need to do this as an idle callback...
  MaterialList materials;
  for (int i = 0; i < mesh->get_surface_count(); ++i) {
    mesh->surface_set_material(i, material);
    materials.push_back(material);
  }

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
  MeshInstance* mesh_instance = Object::cast_to<MeshInstance>(instance);
  if (mesh_instance != nullptr) {
    Mesh* mesh = mesh_instance->get_mesh().ptr();
    if (mesh->get_class_name() == "CylinderMesh" ||
        mesh->get_class_name() == "PlaneMesh") {
      // clang-format off
      basis = Basis(1., 0., 0.,
                    0., 0., -1.,
                    0., 1., 0.) * basis;
      // clang-format on
    }
  }
  instance->set_rotation(basis.get_rotation());
}

void GodotScene::SetInstancePose(int id, const Eigen::Isometry3d& X_WI) {
  Spatial *instance = get_spatial_instance(id);
  SetInstancePose(instance, X_WI);
}

void GodotScene::SetCameraPose(const Eigen::Isometry3d& X_WC) {
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

Ref<SpatialMaterial> GodotScene::MakeSimplePlastic(const Color& color) {
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};

  // Flags
  material->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, false);
  material->set_flag(SpatialMaterial::FLAG_UNSHADED, false);
  material->set_flag(SpatialMaterial::FLAG_USE_VERTEX_LIGHTING, false);
  material->set_flag(SpatialMaterial::FLAG_DISABLE_DEPTH_TEST, false);
  material->set_flag(SpatialMaterial::FLAG_USE_POINT_SIZE, false);
  material->set_flag(SpatialMaterial::FLAG_TRIPLANAR_USE_WORLD, false);
  material->set_flag(SpatialMaterial::FLAG_FIXED_SIZE, false);
  material->set_flag(SpatialMaterial::FLAG_ALBEDO_TEXTURE_FORCE_SRGB, false);

  // Vertex Color
  material->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, false);
  material->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, false);

  // Parameters
  material->set_diffuse_mode(SpatialMaterial::DIFFUSE_BURLEY);
  material->set_specular_mode(SpatialMaterial::SPECULAR_SCHLICK_GGX);
  material->set_blend_mode(SpatialMaterial::BLEND_MODE_MIX);
  material->set_cull_mode(SpatialMaterial::CULL_BACK);
  material->set_depth_draw_mode(SpatialMaterial::DEPTH_DRAW_OPAQUE_ONLY);
  material->set_billboard_mode(SpatialMaterial::BILLBOARD_DISABLED);
  material->set_grow_enabled(false);
  material->set_flag(SpatialMaterial::FLAG_USE_ALPHA_SCISSOR, false);

  // Albedo
  material->set_albedo(Color(color.r, color.g, color.b, 1.0));
  // Metallic
  material->set_metallic(0);
  material->set_specular(0.5);
  // Roughness
  material->set_roughness(1);
  // Features
  material->set_feature(SpatialMaterial::FEATURE_EMISSION, false);
  material->set_feature(SpatialMaterial::FEATURE_NORMAL_MAPPING, false);
  material->set_feature(SpatialMaterial::FEATURE_RIM, false);
  material->set_feature(SpatialMaterial::FEATURE_CLEARCOAT, false);
  material->set_feature(SpatialMaterial::FEATURE_ANISOTROPY, false);
  material->set_feature(SpatialMaterial::FEATURE_AMBIENT_OCCLUSION, false);
  material->set_feature(SpatialMaterial::FEATURE_DEPTH_MAPPING, false);
  material->set_feature(SpatialMaterial::FEATURE_SUBSURACE_SCATTERING, false);
  material->set_feature(SpatialMaterial::FEATURE_TRANSMISSION, false);
  material->set_feature(SpatialMaterial::FEATURE_REFRACTION, false);
  material->set_feature(SpatialMaterial::FEATURE_DETAIL, false);

  SpatialMaterial::flush_changes();

  return material;
}

Ref<SpatialMaterial> GodotScene::MakeLabelMaterial(const Color& color) {
  Ref<SpatialMaterial> material{memnew(SpatialMaterial)};
  material->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
  material->set_albedo(Color(color.r, color.g, color.b, 1.0));
  return material;
}

} // namespace godotvis

