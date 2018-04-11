#include "drake/systems/sensors/rgbd_renderer_godot.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <spruce.hh>
#include <yaml-cpp/yaml.h>

#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "drake/systems/sensors/godot_renderer/godot_scene.h"


namespace drake {
namespace systems {
namespace sensors {

namespace  {
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;
// const std::string path = "/home/kunimatsu/work/godot-demo-projects/3d/material_testers/";
const std::string path = "/home/kunimatsu/dataset/coco/val2014/";
// TODO(SeanCurtis-Tri): This needs to be never_destroyed but with a destructor
// being called. So, it should have a *weak* pointer which returns shared
// pointers.
godotvis::GodotRenderer* Renderer() {
  static godotvis::GodotRenderer renderer(640, 480);
  return &renderer;
}

}  // namespace

class RgbdRendererGodot::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererGodot* parent, const Eigen::Isometry3d& X_WC)
      : parent_(parent), renderer_(Renderer()) {
    scene_.Initialize();
    scene_.set_viewport_size(parent_->config().width, parent_->config().height);
    scene_.AddCamera(parent_->config().fov_y * 180. / M_PI, kClippingPlaneNear,
                     kClippingPlaneFar);

    scene_.SetCameraPose(rotate_to_godot_camera(X_WC));
    // TODO: Setup environment, lighting, GI Probes, etc...
    // Probably from a json config file
    // scene_.SetupEnvironment(path + "night.hdr");
    YAML::Node node = YAML::LoadFile(
        "/home/kunimatsu/work/drake/systems/sensors/bg_image_list.yaml");
    assert(node.IsSequence());
    int index = rand() % 10682;
    auto filename = node[index].as<std::string>();
    scene_.SetupEnvironment(path + filename);

    const auto sky_color =
        ColorPalette::Normalize(parent_->color_palette().get_sky_color());
    scene_.SetBackgroundColor(sky_color.r, sky_color.g, sky_color.b);
  }

  /// Destructor
  ~Impl() {
    scene_.Finish();
  }

  void AddFlatTerrain() {
    // NOTE: Godot is in root namespace.
    // The label and render color is the same.
    ::Color color(GodotColor(
        ColorPalette::Normalize(parent_->color_palette().get_terrain_color())));
    int terrain_id = scene_.AddPlaneInstance(kTerrainSize, kTerrainSize, color,
                                             color);
    MeshInstance* instance = scene_.get_mesh_instance(terrain_id);
    DRAKE_DEMAND(instance != nullptr);
    instance->set_cast_shadows_setting(GeometryInstance::SHADOW_CASTING_SETTING_OFF);
  }

  optional<RgbdRenderer::VisualIndex> RegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id);

  void UpdateVisualPose(const Eigen::Isometry3d& X_WV, int body_id,
                        RgbdRenderer::VisualIndex visual_id) const {
    int godot_id = body_to_godot_ids_.at(body_id).at(visual_id);
    scene_.SetInstancePose(godot_id, X_WV);
    // TODO(SeanCurtis-TRI): Mark scene as dirty so that flush notifcations happens once.
    scene_.FlushTransformNotifications();
  }

  void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const;

  void RenderColorImage(ImageRgba8U* color_image_out) const {
    scene_.ApplyMaterialShader();
    scene_.FlushTransformNotifications();
    renderer_->Draw();
    Ref<::Image> image = scene_.Capture();
    // #define SAVE_COLOR_RENDERS
#ifdef SAVE_COLOR_RENDERS
    static int count = 0;
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(4) << (count++);
    std::string filename = "/home/kunimatsu/Pictures/godot/rgbd_test" +
        ss.str() + ".png";
    std::cout << "save image to: " << filename << std::endl;
    image->save_png(filename.c_str());
#endif
    ConvertGodotImage(color_image_out, image);
    image.unref();
  }

  void RenderDepthImage(ImageDepth32F* depth_image_out) const {
    scene_.ApplyDepthShader();
    scene_.FlushTransformNotifications();
    renderer_->Draw();
    Ref<::Image> image = scene_.Capture();
    ConvertGodotImage(depth_image_out, image);
    image.unref();
  }

  void RenderLabelImage(ImageLabel16I* label_image_out) const {
    scene_.ApplyLabelShader();
    scene_.FlushTransformNotifications();
    renderer_->Draw();
    Ref<::Image> image = scene_.Capture();

    // #define SAVE_LABEL_RENDERS
#ifdef SAVE_LABEL_RENDERS
    static int count = 0;
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(4) << (count++);
    std::string filename = "/home/kunimatsu/Pictures/godot/label_test" +
        ss.str() + ".png";
    std::cout << "save image to: " << filename << std::endl;
    image->save_png(filename.c_str());
#endif

    image->lock();
    ColorI color;
    for (int v = 0; v < parent_->config().height; ++v) {
      for (int u = 0; u < parent_->config().width; ++u) {
        ::Color pixel = image->get_pixel(u, v);
        // NOTE: This relies on float -> int conversion via truncation. It has
        // the effect of rounding c * 255 to the nearest int.
        color.r = static_cast<int>(pixel.r * 255 + 0.5);
        color.g = static_cast<int>(pixel.g * 255 + 0.5);
        color.b = static_cast<int>(pixel.b * 255 + 0.5);
        // Converting an RGB color to an object instance ID.

        // TODO(kunimatsu-tri) Handle this correctly. An exception will be
        // thrown at LookUpId(color) when AddFlatTerrain is not called.
        int label;
        if (color.r == 127 && color.g == 127 && color.b) {
          label = Label::kFlatTerrain;
        } else {
          label = parent_->color_palette().LookUpId(color);
        }
        label_image_out->at(u, v)[0] = static_cast<int16_t>(label);
      }
    }
    image->unlock();
    image.unref();
  }

 private:
  ::Color GodotColor(const Eigen::Vector4d& color) {
    return ::Color(static_cast<float>(color[0]),
                   static_cast<float>(color[1]),
                   static_cast<float>(color[2]),
                   static_cast<float>(color[3]));
  }
  ::Color GodotColor(const ColorD& color) {
    return ::Color(static_cast<float>(color.r),
                   static_cast<float>(color.g),
                   static_cast<float>(color.b));
  }

  void ConvertGodotImage(ImageRgba8U* image_out,
                         Ref<::Image>& image_in) const;

  void ConvertGodotImage(ImageDepth32F* image_out,
                         Ref<::Image>& image_in) const;

  /// Rotate Drake's camera coordinate to Godot's. Drake's camera z axis points
  /// forward, whereas Godot's camera z axis points backward as in OpenGL.
  Eigen::Isometry3d rotate_to_godot_camera(
      const Eigen::Isometry3d& X_WC) const {
    std::cout << "Drake camera pose: " << X_WC.translation().transpose() << std::endl;
    std::cout << X_WC.rotation() << std::endl;
    Eigen::Isometry3d X_WCgodot = X_WC;
    return X_WCgodot.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  }

  RgbdRendererGodot* parent_ = nullptr;
  mutable godotvis::GodotScene scene_;
  /// List of Godot mesh instance indices, each mesh instance corresponds to one
  /// Drake's visual element, added in a specific order.
  using GodotInstanceIds = std::vector<int>;
  /// Map a Drake's body_id to a list of Godot mesh instances, each of which
  /// corresponds to a Drake's visual element associating with this body.
  std::map<int, GodotInstanceIds> body_to_godot_ids_;
  godotvis::GodotRenderer* renderer_{};
};


void RgbdRendererGodot::Impl::ConvertGodotImage(ImageRgba8U* image_out,
                                                Ref<::Image>& godot_image) const {
  godot_image->lock();
  ::Color color;
  for(int y = 0; y < godot_image->get_height(); ++y) {
    for(int x = 0; x < godot_image->get_width(); ++x) {
      color = godot_image->get_pixel(x, y);
      image_out->at(x,y)[0] = std::round(color.r*255);
      image_out->at(x,y)[1] = std::round(color.g*255);
      image_out->at(x,y)[2] = std::round(color.b*255);
      image_out->at(x,y)[3] = std::round(color.a*255);
    }
  }
  godot_image->unlock();
}

void RgbdRendererGodot::Impl::ConvertGodotImage(ImageDepth32F* image_out,
                                                Ref<::Image>& godot_image) const {
  godot_image->lock();
  ::Color color;
  for(int y = 0; y < godot_image->get_height(); ++y) {
    for(int x = 0; x < godot_image->get_width(); ++x) {
      color = godot_image->get_pixel(x, y);
      *image_out->at(x, y) = color.r;
    }
  }
  godot_image->unlock();
}

void RgbdRendererGodot::Impl::UpdateViewpoint(
    const Eigen::Isometry3d& X_WC) const {
  scene_.SetCameraPose(rotate_to_godot_camera(X_WC));
}

// Helper function to tentatively convert an obj file name to a .gltf or .mesh
// file name. First priority is to find the gltf, fall back is mesh. Throws
// an exception in every other case.
std::string ObjToGodot(const DrakeShapes::Mesh& mesh) {
  size_t dot_position(mesh.resolved_filename_.find('.'));
  if (dot_position == std::string::npos) {
    throw std::runtime_error("Mesh file has no extension: " + mesh.uri_);
  }

  std::string source_file = mesh.resolved_filename_;
  std::string extension{mesh.resolved_filename_.substr(dot_position)};
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  if (extension != ".obj" && extension != ".mesh" && extension != ".gltf") {
    throw std::runtime_error("Mesh file has unrecognized extension: " +
                             extension);
  }

  // Try mapping obj, first to gltf, and then to mesh.
  spruce::path source_path{};
  if (extension == ".obj") {
    source_path.setStr(source_file.substr(0, dot_position) + ".gltf");
    if (source_path.exists()) {
      return source_path.getStr();
    }
    source_path.setStr(source_file.substr(0, dot_position) + ".mesh");
    if (source_path.exists()) {
      return source_path.getStr();
    }
    throw std::runtime_error(
        "Couldn't find a .mesh or .gltf file for the specified obj file: " +
        source_file);
  }

  return source_file;
}

optional<RgbdRenderer::VisualIndex> RgbdRendererGodot::Impl::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  int godot_id = -1;

  // NOTE: Godot is in root namespace.
  ::Color label_color(
      GodotColor(parent_->color_palette().get_normalized_color(body_id)));
  ::Color color(GodotColor(visual.getMaterial()));

  switch (visual.getShape()) {
    case DrakeShapes::BOX: {
      const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      godot_id = scene_.AddCubeInstance(box.size(0), box.size(1), box.size(2),
                                        color, label_color);
      break;
    }
    case DrakeShapes::SPHERE: {
      const auto& sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      godot_id = scene_.AddSphereInstance(sphere.radius, color, label_color);
      break;
    }
    case DrakeShapes::CYLINDER: {
      const auto& cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      godot_id = scene_.AddCylinderInstance(cylinder.length, cylinder.radius,
                                            color, label_color);
      break;
    }
    case DrakeShapes::MESH: {
      // Swap obj for mesh, test if the file exists, load it.
      const auto& mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);


      godot_id = scene_.AddMeshInstance(ObjToGodot(mesh), color, label_color);
      scene_.SetInstanceScale(godot_id, mesh.scale_(0), mesh.scale_(1),
                              mesh.scale_(2));
      break;
    }
    default:
      break;
  }

  if (godot_id > 0) {
    body_to_godot_ids_[body_id].push_back(godot_id);
    int visual_id = static_cast<int>(body_to_godot_ids_[body_id].size() - 1);
    return optional<VisualIndex>(VisualIndex(visual_id));
  }
  return nullopt;
}

RgbdRendererGodot::RgbdRendererGodot(const RenderingConfig& config,
                                     const Eigen::Isometry3d& X_WC)
    : RgbdRenderer(config, X_WC),
      impl_(new RgbdRendererGodot::Impl(this, X_WC)) {}

RgbdRendererGodot::~RgbdRendererGodot() {}

optional<RgbdRenderer::VisualIndex> RgbdRendererGodot::ImplRegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return impl_->RegisterVisual(visual, body_id);
}

void RgbdRendererGodot::ImplAddFlatTerrain() { impl_->AddFlatTerrain(); }

void RgbdRendererGodot::ImplUpdateViewpoint(
    const Eigen::Isometry3d& X_WR) const {
  impl_->UpdateViewpoint(X_WR);
}

void RgbdRendererGodot::ImplUpdateVisualPose(
    const Eigen::Isometry3d& X_WV, int body_id,
    RgbdRenderer::VisualIndex visual_id) const {
  impl_->UpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRendererGodot::ImplRenderColorImage(
    ImageRgba8U* color_image_out) const {
  impl_->RenderColorImage(color_image_out);
}

void RgbdRendererGodot::ImplRenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  impl_->RenderDepthImage(depth_image_out);
}

void RgbdRendererGodot::ImplRenderLabelImage(
    ImageLabel16I* label_image_out) const {
  impl_->RenderLabelImage(label_image_out);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
