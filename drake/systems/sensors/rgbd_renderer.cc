#include "drake/systems/sensors/rgbd_renderer.h"

#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLTexture.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOBJReader.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/depth_shaders.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"

// This macro declares vtkRenderingOpenGL2_AutoInit_{Construct(), Destruct()}
// functions and the former is called via VTK_AUTOINIT_CONSTRUCT in
// ModuleInitVtkRenderingOpenGL2.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)

// TODO(kunimatsu-tri) Refactor RgbdRenderer with GeometryWorld when it's ready,
// so that other VTK dependent sensor simulators can share the world without
// duplicating it.

namespace drake {
namespace systems {
namespace sensors {

using vtk_util::ConvertToVtkTransform;
using vtk_util::MakeVtkPointerArray;

namespace {

const int kNumMaxLabel = 256;

// TODO(kunimatsu-tri) Add support for the arbitrary clipping planes.
const double kClippingPlaneNear = 0.01;
const double kClippingPlaneFar = 100.;
const double kTerrainSize = 100.;

// For Zbuffer value conversion.
// const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
// const double kB = -kA * kClippingPlaneNear;


template <typename T>
using ImageSetHelper = std::array<T, 3>;

enum ImageType {
  kColor = 0,
  kDepth = 1,
  kLabel = 2,
};

using ActorCollection = std::vector<vtkSmartPointer<vtkActor>>;

struct RenderingPipeline {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  vtkNew<vtkWindowToImageFilter> filter;
  vtkNew<vtkImageExport> exporter;
};

// Updates VTK rendering related objects including vtkRenderWindow,
// vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
// vtkActors' pose update for rendering.
void PerformVTKUpdate(const std::unique_ptr<RenderingPipeline>& p) {
  p->window->Render();
  p->filter->Modified();
  p->filter->Update();
  p->exporter->Update();
}

// Register the object factories for the vtkRenderingOpenGL2 module.
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2() {
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_ABORT_MSG("File has no extention.");
  }
  return filepath.substr(0, last_dot);
}

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  camera->SetFocalPoint(0., 0., 1.);  // Sets z-forward.
  camera->SetViewUp(0., -1, 0.);  // Sets y-down. For the detail, please refer
  // to CameraInfo's document.
  camera->ApplyTransform(X_WC);
}

class vtkShaderCallback : public vtkCommand {
 public:
  static vtkShaderCallback* New() { return new vtkShaderCallback; }

  void Execute(vtkObject*, unsigned long, void* callback_object) VTK_OVERRIDE {
    vtkOpenGLHelper* cell_bo =
        reinterpret_cast<vtkOpenGLHelper*>(callback_object);
    cell_bo->Program->SetUniformf("z_near", z_near_);
    cell_bo->Program->SetUniformf("z_far", z_far_);
    cell_bo = nullptr;
  }

  void set_renderer(vtkRenderer* renderer) { renderer_ = renderer; }

  void set_z_near(float z_near) {
    z_near_ = z_near;
  }

  void set_z_far(float z_far) {
    z_far_ = z_far;
  }

  vtkShaderCallback() { this->renderer_ = nullptr; }

 private:
  vtkRenderer *renderer_;
  float z_near_{0.f};
  float z_far_{0.f};
};


}  // namespace

class RgbdRenderer::Impl : private ModuleInitVtkRenderingOpenGL2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(const Eigen::Isometry3d& X_WC, int width, int height,
       double z_near, double z_far, double fov_y, bool show_window);

  ~Impl() {}

  void AddFlatTerrain();

  optional<VisualIndex> RegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id);

  void UpdateVisualPose(
      const Eigen::Isometry3d& X_WV, int body_id, VisualIndex visual_id) const;

  void UpdateViewpoint(const Eigen::Isometry3d& X_WR) const;

  void RenderColorImage(ImageRgba8U* color_image_out) const;

  void RenderDepthImage(ImageDepth32F* depth_image_out) const;

  void RenderLabelImage(ImageLabel16I* label_image_out) const;

  int width() const { return width_; }

  int height() const { return height_; }

  double fov_y() const { return fov_y_; }

  const ColorI& get_sky_color() const {
    return color_palette_.get_sky_color();
  }

  const ColorI& get_flat_terrain_color() const {
    return color_palette_.get_terrain_color();
  }

 private:
  float CheckRangeAndConvertToMeters(uint8_t z_buffer_value) const;
  const int width_;
  const int height_;
  const double fov_y_;
  const double z_near_;
  const double z_far_;
  const ColorPalette color_palette_;

  vtkNew<vtkActor> terrain_actor_;
  vtkNew<vtkActor> terrain_depth_actor_;

  ImageSetHelper<std::unique_ptr<RenderingPipeline>> pipelines_;

  // A map which takes pairs of a body index in RBT and three vectors of
  // vtkSmartPointer to vtkActor for color, depth and label rendering
  // respectively. The each vtkActor corresponds to an visual
  // element specified in SDF / URDF.
  // std::map<int, ActorCollections> id_object_maps_;

  std::map<int, ImageSetHelper<ActorCollection>> id_object_maps_;

  vtkNew<vtkImageData> noise_texture_;
  vtkNew<vtkShaderCallback> callback_;
};

void RgbdRenderer::Impl::AddFlatTerrain() {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtk_util::CreateSquarePlane(kTerrainSize);
  vtkNew<vtkOpenGLPolyDataMapper> mapper;
  mapper->SetInputConnection(plane->GetOutputPort());
  terrain_actor_->SetMapper(mapper.GetPointer());
  auto color = ColorPalette::Normalize(color_palette_.get_terrain_color());
  terrain_actor_->GetProperty()->SetColor(color.r, color.g, color.b);
  terrain_actor_->GetProperty()->LightingOff();
  pipelines_[ImageType::kColor]->renderer->AddActor(
      terrain_actor_.GetPointer());
  pipelines_[ImageType::kLabel]->renderer->AddActor(
      terrain_actor_.GetPointer());

  vtkNew<vtkOpenGLPolyDataMapper> depth_mapper;
  depth_mapper->SetVertexShaderCode(shaders::kDepthVS);
  depth_mapper->SetFragmentShaderCode(shaders::kDepthFS);
  depth_mapper->SetInputConnection(plane->GetOutputPort());
  terrain_depth_actor_->SetMapper(depth_mapper.GetPointer());
  pipelines_[ImageType::kDepth]->renderer->AddActor(
      terrain_depth_actor_.GetPointer());
}

void RgbdRenderer::Impl::UpdateVisualPose(
    const Eigen::Isometry3d& X_WV, int body_id, VisualIndex visual_id) const {
  vtkSmartPointer<vtkTransform> vtk_X_WV = ConvertToVtkTransform(X_WV);
  // `id_object_maps_` is modified here. This is OK because 1) we are just
  // copying data to the memory spaces allocated at construction time
  // and 2) we are not outputting these data to outside the class.
  auto& actor_collections = id_object_maps_.at(body_id);
  for (auto& actor_collection : actor_collections) {
    actor_collection.at(visual_id)->SetUserTransform(vtk_X_WV);
  }
}

void RgbdRenderer::Impl::UpdateViewpoint(const Eigen::Isometry3d& X_WR) const {
  vtkSmartPointer<vtkTransform> vtk_X_WR = ConvertToVtkTransform(X_WR);
  for (auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WR);
  }
}

void RgbdRenderer::Impl::RenderColorImage(
    ImageRgba8U* color_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(pipelines_[ImageType::kColor]);
  pipelines_[ImageType::kColor]->exporter->Export(color_image_out->at(0, 0));
}

void RgbdRenderer::Impl::RenderDepthImage(
    ImageDepth32F* depth_image_out) const {
  // TODO(kunimatsu-tri) Clean up here.
  std::srand(std::time(0));
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      float r = static_cast<float>(std::rand() / static_cast<float>(RAND_MAX));
      float* pixel =
          static_cast<float*>(noise_texture_->GetScalarPointer(u, v, 0));
      pixel[0] = r;
    }
  }

  vtkNew<vtkOpenGLTexture> texture;
  texture->SetInputData(noise_texture_.GetPointer());
  for (auto& kv : id_object_maps_) {
    auto& depth_actor_collection = kv.second[ImageType::kDepth];
    for (auto& actor : depth_actor_collection) {
      actor->SetTexture(texture.Get());
    }
  }

  ImageRgba8U image(width_, height_);
  // TODO(sherm1) Should evaluate VTK cache entry.
  PerformVTKUpdate(pipelines_[ImageType::kDepth]);
  pipelines_[ImageType::kDepth]->exporter->Export(image.at(0, 0));

  std::cout << static_cast<int>(image.at(0, 0)[0]) << std::endl;
  std::cout << static_cast<int>(image.at(0, 0)[1]) << std::endl;
  std::cout << static_cast<int>(image.at(0, 0)[2]) << std::endl;
  // TODO(kunimatsu-tri) Calculate this in a vertex shader.
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      depth_image_out->at(u, v)[0] =
          CheckRangeAndConvertToMeters(image.at(u, v)[0]);
    }
  }
}

void RgbdRenderer::Impl::RenderLabelImage(
    ImageLabel16I* label_image_out) const {
  // TODO(sherm1) Should evaluate VTK cache entry.
  ImageRgba8U image(width_, height_);
  PerformVTKUpdate(pipelines_[ImageType::kLabel]);
  pipelines_[ImageType::kLabel]->exporter->Export(image.at(0, 0));
  ColorI color;
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      color.r = image.at(u, v)[0];
      color.g = image.at(u, v)[1];
      color.b = image.at(u, v)[2];
      label_image_out->at(u, v)[0] =
          static_cast<int16_t>(color_palette_.LookUpId(color));
    }
  }
}

RgbdRenderer::Impl::Impl(const Eigen::Isometry3d& X_WC,
                         int width,
                         int height,
                         double z_near,
                         double z_far,
                         double fov_y,
                         bool show_window)
    : width_(width), height_(height), fov_y_(fov_y),
      z_near_(z_near), z_far_(z_far),
      color_palette_(kNumMaxLabel, Label::kFlatTerrain, Label::kNoBody),
      pipelines_{{
          std::make_unique<RenderingPipeline>(),
          std::make_unique<RenderingPipeline>(),
          std::make_unique<RenderingPipeline>()}} {
  if (show_window) {
    pipelines_[ImageType::kColor]->window->SetWindowName("Color Image");
    pipelines_[ImageType::kDepth]->window->SetWindowName("Depth Image");
    pipelines_[ImageType::kLabel]->window->SetWindowName("Label Image");
  } else {
    for (auto& pipeline : pipelines_) {
      pipeline->window->SetOffScreenRendering(1);
    }
  }

  const auto sky_color =
      ColorPalette::Normalize(color_palette_.get_sky_color());
  const vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  pipelines_[ImageType::kLabel]->window->SetMultiSamples(0);

  for (auto& pipeline : pipelines_) {
    pipeline->renderer->SetBackground(sky_color.r, sky_color.g, sky_color.b);
    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(fov_y_ * 180. / M_PI);
    camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);

    pipeline->window->SetSize(width_, height_);
    pipeline->window->AddRenderer(pipeline->renderer.GetPointer());
    pipeline->filter->SetInput(pipeline->window.GetPointer());
    pipeline->filter->SetMagnification(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->filter->Update();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();
  }

  {
    auto camera = pipelines_[ImageType::kDepth]->renderer->GetActiveCamera();
    camera->SetClippingRange(z_near, z_far_);
    pipelines_[ImageType::kDepth]->renderer->SetBackground(1., 1., 1.);
  }
  pipelines_[ImageType::kColor]->renderer->SetUseDepthPeeling(1);
  pipelines_[ImageType::kColor]->renderer->UseFXAAOn();

  callback_->set_renderer(pipelines_[ImageType::kDepth]->renderer.Get());
  callback_->set_z_near(z_near_);
  callback_->set_z_far(z_far_);

  noise_texture_->SetDimensions(width_, height_, 1);
  noise_texture_->AllocateScalars(VTK_FLOAT, 1);
}

optional<RgbdRenderer::VisualIndex> RgbdRenderer::Impl::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  ImageSetHelper<vtkNew<vtkActor>> actors;
  ImageSetHelper<vtkNew<vtkOpenGLPolyDataMapper>> mappers;
  // Sets vertex and fragment shaders only to the depth mapper.
  mappers[ImageType::kDepth]->SetVertexShaderCode(shaders::kDepthVS);
  mappers[ImageType::kDepth]->SetFragmentShaderCode(shaders::kDepthFS);
  mappers[ImageType::kDepth]->AddObserver(
      vtkCommand::UpdateShaderEvent, callback_.Get());

  bool shape_matched = true;
  const DrakeShapes::Geometry& geometry = visual.getGeometry();
  switch (visual.getShape()) {
    case DrakeShapes::BOX: {
      auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      vtkNew<vtkCubeSource> vtk_cube;
      vtk_cube->SetXLength(box.size(0));
      vtk_cube->SetYLength(box.size(1));
      vtk_cube->SetZLength(box.size(2));

      for (auto& mapper : mappers) {
        mapper->SetInputConnection(vtk_cube->GetOutputPort());
      }

      break;
    }
    case DrakeShapes::SPHERE: {
      auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      vtkNew<vtkSphereSource> vtk_sphere;
      vtk_sphere->SetRadius(sphere.radius);
      vtk_sphere->SetThetaResolution(50);
      vtk_sphere->SetPhiResolution(50);

      for (auto& mapper : mappers) {
        mapper->SetInputConnection(vtk_sphere->GetOutputPort());
      }

      break;
    }
    case DrakeShapes::CYLINDER: {
      auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      vtkNew<vtkCylinderSource> vtk_cylinder;
      vtk_cylinder->SetHeight(cylinder.length);
      vtk_cylinder->SetRadius(cylinder.radius);
      vtk_cylinder->SetResolution(50);

      // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
      // to rotate it to be z-axis aligned because that is what Drake uses.
      vtkNew<vtkTransform> transform;
      transform->RotateX(90);
      vtkNew<vtkTransformPolyDataFilter> transform_filter;
      transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
      transform_filter->SetTransform(transform.GetPointer());
      transform_filter->Update();

      for (auto& mapper : mappers) {
        mapper->SetInputConnection(transform_filter->GetOutputPort());
      }

      break;
    }
    case DrakeShapes::MESH: {
      const auto& mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
      const auto* mesh_filename = mesh.resolved_filename_.c_str();
      // TODO(kunimatsu-tri) Add support for other file formats.
      vtkNew<vtkOBJReader> mesh_reader;
      mesh_reader->SetFileName(mesh_filename);
      mesh_reader->Update();

      const double scale = mesh.scale_[0];
      vtkNew<vtkTransform> transform;
      transform->Scale(scale, scale, scale);
      vtkNew<vtkTransformPolyDataFilter> transform_filter;
      transform_filter->SetInputConnection(mesh_reader->GetOutputPort());
      transform_filter->SetTransform(transform.GetPointer());
      transform_filter->Update();

      for (auto& mapper : mappers) {
        mapper->SetInputConnection(transform_filter->GetOutputPort());
      }

      // TODO(kunimatsu-tri) Guessing the texture file name is bad. Instead,
      // get it from somewhere like `DrakeShapes::MeshWithTexture` when it's
      // implemented.
      // TODO(kunimatsu-tri) Add support for other file formats.
      const std::string texture_file(
          RemoveFileExtension(mesh_filename) + ".png");
      std::ifstream file_exist(texture_file);

      if (file_exist) {
        vtkNew<vtkPNGReader> texture_reader;
        texture_reader->SetFileName(texture_file.c_str());
        texture_reader->Update();
        vtkNew<vtkOpenGLTexture> texture;
        texture->SetInputConnection(texture_reader->GetOutputPort());
        texture->InterpolateOn();
        actors[ImageType::kColor]->SetTexture(texture.Get());
      }

      break;
    }
    case DrakeShapes::CAPSULE: {
      // TODO(kunimatsu-tri) Implement this as needed.
      shape_matched = false;
      break;
    }
    default: {
      shape_matched = false;
      break;
    }
  }

  // Registers actors.
  if (shape_matched) {
    auto& color_actor = actors[ImageType::kColor];
    if (color_actor->GetProperty()->GetNumberOfTextures() == 0) {
      const auto color = visual.getMaterial();
      color_actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    }

    auto& label_actor = actors[ImageType::kLabel];
    // This is to disable shadows and to get an object painted with a single
    // color.
    label_actor->GetProperty()->LightingOff();
    const auto& color = color_palette_.get_normalized_color(body_id);
    label_actor->GetProperty()->SetColor(color.r, color.g, color.b);

    vtkSmartPointer<vtkTransform> vtk_transform =
        ConvertToVtkTransform(visual.getWorldTransform());

    auto& actor_collections = id_object_maps_[body_id];
    for (size_t i = 0; i < actors.size(); ++i) {
      actors[i]->SetMapper(mappers[i].GetPointer());
      actors[i]->SetUserTransform(vtk_transform);
      pipelines_[i]->renderer->AddActor(actors[i].GetPointer());
      actor_collections[i].push_back(actors[i].GetPointer());
    }

    return optional<VisualIndex>(VisualIndex(static_cast<int>(
        id_object_maps_[body_id][ImageType::kColor].size() - 1)));
  }

  return nullopt;
}

float RgbdRenderer::Impl::CheckRangeAndConvertToMeters(
    uint8_t z_buffer_value) const {
  float z;
  // When the depth is either closer than `kClippingPlaneNear` or farther than
  // `kClippingPlaneFar`, `z_buffer_value` becomes `255`.
  if (z_buffer_value == 255u) {
    z = std::numeric_limits<float>::quiet_NaN();
  } else if (z_buffer_value == 0u) {
    z = InvalidDepth::kTooClose;
  } else {
    z = static_cast<float>(z_buffer_value)
        / 255.f * (z_far_ - z_near_) + z_near_;
    if (z > z_far_) {
      z = InvalidDepth::kTooFar;
    } else if (z < z_near_) {
      z = InvalidDepth::kTooClose;
    }
  }

  return z;
}

RgbdRenderer::RgbdRenderer(const Eigen::Isometry3d& X_WC,
                           int width,
                           int height,
                           double z_near,
                           double z_far,
                           double fov_y,
                           bool show_window)
    : impl_(new RgbdRenderer::Impl(X_WC, width, height, z_near, z_far, fov_y,
                                   show_window)) {}

RgbdRenderer::~RgbdRenderer() {}

optional<RgbdRenderer::VisualIndex> RgbdRenderer::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return impl_->RegisterVisual(visual, body_id);
}

void RgbdRenderer::AddFlatTerrain() {
  impl_->AddFlatTerrain();
}

void RgbdRenderer::UpdateViewpoint(const Eigen::Isometry3d& X_WR) const {
  impl_->UpdateViewpoint(X_WR);
}

void RgbdRenderer::UpdateVisualPose(
    const Eigen::Isometry3d& X_WV, int body_id, VisualIndex visual_id) const {
  impl_->UpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRenderer::RenderColorImage(ImageRgba8U* color_image_out) const {
  impl_->RenderColorImage(color_image_out);
}

void RgbdRenderer::RenderDepthImage(ImageDepth32F* depth_image_out) const {
  impl_->RenderDepthImage(depth_image_out);
}

void RgbdRenderer::RenderLabelImage(ImageLabel16I* label_image_out) const {
  impl_->RenderLabelImage(label_image_out);
}

int RgbdRenderer::width() const { return impl_->width(); }

int RgbdRenderer::height() const { return impl_->height(); }

double RgbdRenderer::fov_y() const { return impl_->fov_y(); }

const ColorI& RgbdRenderer::get_sky_color() const {
  return impl_->get_sky_color();
}

const ColorI& RgbdRenderer::get_flat_terrain_color() const {
  return impl_->get_flat_terrain_color();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
