#include "drake/systems/sensors/rgbd_renderer_godot.h"

#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "drake/systems/sensors/godot_renderer/godot_scene.h"


namespace drake {
namespace systems {
namespace sensors {

namespace  {
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;
const std::string path = "/home/sean/code/godot-demo-projects/3d/material_testers/";

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
    scene_.SetupEnvironment(path + "night.hdr");

    const auto sky_color =
        ColorPalette::Normalize(parent_->color_palette().get_sky_color());
    scene_.SetBackgroundColor(sky_color.r, sky_color.g, sky_color.b);
  }

  /// Destructor
  ~Impl() {
    scene_.Finish();
  }

  void AddFlatTerrain() {
    int plane_id = scene_.AddPlaneInstance(kTerrainSize, kTerrainSize);
    auto color =
        ColorPalette::Normalize(parent_->color_palette().get_terrain_color());
    scene_.SetInstanceColor(plane_id, color.r, color.g, color.b);
  }

  optional<RgbdRenderer::VisualIndex> RegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id);

  void UpdateVisualPose(const Eigen::Isometry3d& X_WV, int body_id,
                        RgbdRenderer::VisualIndex visual_id) const {}

  void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const;

  void RenderColorImage(ImageRgba8U* color_image_out) const {
//    static int count = 0;
    scene_.ApplyMaterialShader();
    scene_.FlushTransformNotifications();
    renderer_->Draw();
    Ref<::Image> image = scene_.Capture();
//    std::string filename = "/home/sean/Pictures/godot/rgbd_test" +
//        std::to_string(count++) + ".png";
//    std::cout << "save image to: " << filename << std::endl;
//    image->save_png(filename.c_str());
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
    const int w = parent_->config().width;
    const int h = parent_->config().height;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        // NOTE: The value "2" here is to accommodate the
        // rgbd_camera_rendering_example.cc code; a value of terrain or 1 is
        // ignored. 2 triggers the logic that indicates that *something* is
        // visible.
        label_image_out->at(x, y)[0] = static_cast<int16_t>(2);
      }
    }
//    throw std::runtime_error(
//        "Godot renderer does not support label images yet!");
  }

 private:
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

optional<RgbdRenderer::VisualIndex> RgbdRendererGodot::Impl::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  std::cerr << "\n!!! RgbdRendererGodot::Impl::RegisterVisual\n";
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  int godot_id = -1;
  switch (visual.getShape()) {
    case DrakeShapes::BOX: {
      const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      godot_id = scene_.AddCubeInstance(box.size(0), box.size(1), box.size(2));
      const auto& color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;
    }
    case DrakeShapes::SPHERE: {
      const auto& sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      godot_id = scene_.AddSphereInstance(sphere.radius);
      auto color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;
    }
    case DrakeShapes::CYLINDER: {
      const auto& cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      godot_id = scene_.AddCylinderInstance(cylinder.length, cylinder.radius);
      const auto& color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;
    }
    case DrakeShapes::MESH: {
      godot_id = scene_.AddCubeInstance(0.02, 0.02, 0.1);
      const auto& color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;

//      auto mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
//      std::cerr << "LOading: " << mesh.resolved_filename_ << "\n";
//      godot_id = scene_.AddMeshInstance(mesh.resolved_filename_);
//      auto color = visual.getMaterial();
//      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);


      // TODO(duy) Use gltf in Drake by default?
      //const auto* mesh_filename =
      //dynamic_cast<const DrakeShapes::Mesh&>(geometry)
      //.resolved_filename_.c_str();
      //const std::string mesh_gltf(RemoveFileExtension(mesh_filename) + ".gltf");
      //godot_id = scene_.AddMeshInstance(mesh_gltf);
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
