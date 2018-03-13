//#include "drake/systems/sensors/rgbd_renderer_godot.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/test/rgbd_renderer_test_util.h"

#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "drake/systems/sensors/godot_renderer/godot_scene.h"

namespace drake {
namespace systems {
namespace sensors {

namespace {
// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
// TODO(sean) unify these same constants in rgbd_renderer_vtk.cc
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;
}

class RgbdRendererGodot final : public RgbdRenderer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdRendererGodot)

  RgbdRendererGodot(
      const RenderingConfig& config,
      const Eigen::Isometry3d& X_WC = Eigen::Isometry3d::Identity());

  ~RgbdRendererGodot();

 private:
  void ImplAddFlatTerrain() override;

  optional<RgbdRenderer::VisualIndex> ImplRegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id) override;

  void ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV, int body_id,
                            RgbdRenderer::VisualIndex visual_id) const override;

  void ImplUpdateViewpoint(const Eigen::Isometry3d& X_WC) const override;

  void ImplRenderColorImage(ImageRgba8U* color_image_out) const override;

  void ImplRenderDepthImage(ImageDepth32F* depth_image_out) const override;

  void ImplRenderLabelImage(ImageLabel16I* label_image_out) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

// TODO(duy): Proper singleton
godotvis::GodotRenderer godot_renderer(640, 480);
// TODO(duy): remove this hacky path
//std::string path = "/home/duynguyen/git/godot-demo-projects/3d/material_testers/";
std::string path = "/home/sean/code/godot-demo-projects/3d/material_testers/";

class RgbdRendererGodot::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(RgbdRendererGodot* parent, const Eigen::Isometry3d& X_WC)
      : parent_(parent) {
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
     static int count = 0;
     scene_.ApplyMaterialShader();
     scene_.FlushTransformNotifications();
     godot_renderer.Draw();
     Ref<::Image> image = scene_.Capture();
     std::string filename = "/home/sean/Downloads/rgbd_test/" +
                            std::to_string(count++) + ".png";
     std::cout << "save image to: " << filename << std::endl;
     image->save_png(filename.c_str());
     ConvertGodotImage(color_image_out, image);
     image.unref();
  }

  void RenderDepthImage(ImageDepth32F* depth_image_out) const {
     scene_.ApplyDepthShader();
     scene_.FlushTransformNotifications();
     godot_renderer.Draw();
     Ref<::Image> image = scene_.Capture();
     ConvertGodotImage(depth_image_out, image);
     image.unref();
  }

  void RenderLabelImage(ImageLabel16I* label_image_out) const {
    throw std::runtime_error(
        "Godot renderer does not support label images yet!");
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
  /// Map a Drake's body_id to a list of Godot mesh intances, each of which
  /// corresponds to a Drake's visual element associating with this body.
  std::map<int, GodotInstanceIds> body_to_godot_ids_;
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
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  int godot_id = -1;
  switch (visual.getShape()) {
    case DrakeShapes::BOX: {
      auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      godot_id = scene_.AddCubeInstance(box.size(0), box.size(1), box.size(2));
      auto color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;
    }
    case DrakeShapes::SPHERE: {
      auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      godot_id = scene_.AddSphereInstance(sphere.radius);
      auto color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;
    }
    case DrakeShapes::CYLINDER: {
      auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      godot_id = scene_.AddCylinderInstance(cylinder.length, cylinder.radius);
      auto color = visual.getMaterial();
      scene_.SetInstanceColor(godot_id, color[0], color[1], color[2]);
      break;
    }
    case DrakeShapes::MESH: {
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
    return optional<VisualIndex>(VisualIndex(static_cast<int>(visual_id)));
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

/**************************************************************************/
namespace test {

using RgbdRendererGodotTest = RgbdRendererTest<RgbdRendererGodot>;

using Eigen::Isometry3d;

TEST_F(RgbdRendererGodotTest, InstantiationTest) {
  Init(Isometry3d::Identity());

  EXPECT_EQ(renderer_->config().width, kWidth);
  EXPECT_EQ(renderer_->config().height, kHeight);
  EXPECT_EQ(renderer_->config().fov_y, kFovY);
  // TODO(duy): Actually check these params inside Impl::scene_'s viewport
}

TEST_F(RgbdRendererGodotTest, DISABLED_NoBodyTest) {
  // NOTE: This test cannot pass in the configuration in which the environment
  // is not a simple solid color -- loading night.hdr means an empty scene will
  // be some portion of the hdr image.
  Init(Isometry3d::Identity());
  RenderColorImage();

  VerifyUniformColor(renderer_->color_palette().get_sky_color(), 255u);
}

TEST_F(RgbdRendererGodotTest, TerrainTest) {
  Init(X_WC_, true);
  RenderColorImage();

  const auto& kTerrain = renderer_->color_palette().get_terrain_color();
  //const auto& kSky = renderer_->color_palette().get_sky_color();

  // At two different distances.
  for (float depth : {2.f, 5.f}) {
    X_WC_.translation().z() = depth;
    renderer_->UpdateViewpoint(X_WC_);
    RenderColorImage();
    VerifyUniformColor(kTerrain, 255u);
  }

  // Closer than kZNear.
  X_WC_.translation().z() = kZNear - 1e-5;
  renderer_->UpdateViewpoint(X_WC_);
  RenderColorImage();
  //VerifyUniformColor(kTerrain, 255u);

  // Farther than kZFar.
  X_WC_.translation().z() = kZFar + 1e-3;
  renderer_->UpdateViewpoint(X_WC_);
  RenderColorImage();
  //VerifyUniformColor(kTerrain, 255u);
}

TEST_F(RgbdRendererGodotTest, HorizonTest) {
  // Camera at the origin, pointing in a direction parallel to the ground.
  Isometry3d X_WC =
      Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  Init(X_WC, true);

  // Returns y in [0, kHeight / 2], index of horizon location in image
  // coordinate system under two assumptions: 1) the gound plane is not clipped
  // by `kClippingPlaneFar`, 2) camera is located above the ground.
  auto CalcHorizon = [](double z, double fov, double height) {
    const double kTerrainSize = 50.;
    const double kFocalLength = height * 0.5 / std::tan(0.5 * fov);
    return 0.5 * height + z / kTerrainSize * kFocalLength;
  };

  // Verifies v index of horizon at three different camera heights.
  for (double z : {2., 1., 0.5}) {
    X_WC.translation().z() = z;
    renderer_->UpdateViewpoint(X_WC);
    RenderColorImage();

    const auto& kTerrain = renderer_->color_palette().get_terrain_color();
    int actual_horizon{0};
    for (int y = 0; y < kHeight; ++y) {
      // Looking for the boundary between the sky and the ground.
      if ((static_cast<uint8_t>(kTerrain.r == color_.at(0, y)[0])) &&
          (static_cast<uint8_t>(kTerrain.g == color_.at(0, y)[1])) &&
          (static_cast<uint8_t>(kTerrain.b == color_.at(0, y)[2]))) {
        actual_horizon = y;
        break;
      }
    }

    const double expected_horizon = CalcHorizon(z, kFovY, kHeight);
    ASSERT_NEAR(expected_horizon, actual_horizon, 1.001);
  }
}

TEST_F(RgbdRendererGodotTest, BoxTest) {
  Init(X_WC_, true);

  // Sets up a box.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  Eigen::Vector3d box_size(1, 1, 1);
  visual.setGeometry(DrakeShapes::Box(box_size));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  RenderColorImage();

  VerifyOutliers(true);

  // Verifies inside the box.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u,
               kColorPixelTolerance);
  // Depth
  //ASSERT_NEAR(depth_.at(x, y)[0], 2.f, kDepthTolerance);
  // Label
  //ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

TEST_F(RgbdRendererGodotTest, SphereTest) {
  Init(X_WC_, true);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.5;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Sphere(0.5));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  RenderColorImage();

  VerifyOutliers(true);

  // Verifies inside the sphere.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u,
               kColorPixelTolerance);
  // Depth
  //ASSERT_NEAR(depth_.at(x, y)[0], 2.f, kDepthTolerance);
  // Label
  //ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

TEST_F(RgbdRendererGodotTest, CylinderTest) {
  Init(X_WC_, true);

  // Sets up a sphere.
  Isometry3d X_WV = Isometry3d::Identity();
  X_WV.translation().z() = 0.6;
  DrakeShapes::VisualElement visual(X_WV);
  visual.setGeometry(DrakeShapes::Cylinder(0.2, 1.2));  // Radius and length.
  const int kBodyID = 1;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  RenderColorImage();

  VerifyOutliers(true);

  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), kDefaultVisualColor, 255u,
               kColorPixelTolerance);
  // Depth
  //ASSERT_NEAR(depth_.at(x, y)[0], 1.8f, kDepthTolerance);
  // Label
  //ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

TEST_F(RgbdRendererGodotTest, DISABLED_MeshTest) {
  Init(X_WC_, true);

  Isometry3d X_WV = Isometry3d::Identity();
  DrakeShapes::VisualElement visual(X_WV);
  auto filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");
  visual.setGeometry(DrakeShapes::Mesh("", filename));
  const int kBodyID = 0;
  const RgbdRenderer::VisualIndex kVisualID(0);
  renderer_->RegisterVisual(visual, kBodyID);
  renderer_->UpdateVisualPose(X_WV, kBodyID, kVisualID);
  RenderColorImage();

  VerifyOutliers(true);

  // Verifies inside the cylinder.
  const int x = kInlier.x;
  const int y = kInlier.y;
  // Color
  CompareColor(color_.at(x, y), ColorI({4u, 241u, 33u}), 255u,
               kColorPixelTolerance);
  // Depth
  //ASSERT_NEAR(depth_.at(x, y)[0], 2., kDepthTolerance);
  // Label
  //ASSERT_EQ(label_.at(x, y)[0], kBodyID);
}

}  // namespace test
}  // namespace sensors
}  // namespace systems
}  // namespace drake
