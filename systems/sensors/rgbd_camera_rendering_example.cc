#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>

#include "drake/common/unused.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_camera.h"

using std::cout;
using std::endl;
using std::string;

using drake::multibody::joints::kQuaternion;
using drake::multibody::joints::kFixed;

namespace drake {
namespace systems {
namespace sensors {
namespace {

template <PixelType kPixelType>
void SaveToFile(const string& filepath, const Image<kPixelType>& image) {
  const int width = image.width();
  const int height = image.height();
  const int num_channels = Image<kPixelType>::kNumChannels;

  vtkNew<vtkImageData> vtk_image;
  vtk_image->SetDimensions(width, height, 1);

  switch (kPixelType) {
    case PixelType::kRgba8U:
      vtk_image->AllocateScalars(VTK_UNSIGNED_CHAR, num_channels);
      break;
    case PixelType::kDepth32F:
      vtk_image->AllocateScalars(VTK_FLOAT, num_channels);
      break;
    case PixelType::kLabel16I:
      vtk_image->AllocateScalars(VTK_UNSIGNED_SHORT, num_channels);
      break;
  }

  auto image_ptr = reinterpret_cast<
    typename Image<kPixelType>::T*>(vtk_image->GetScalarPointer());
  const int num_scalar_components = vtk_image->GetNumberOfScalarComponents();

  for (int v = height - 1; v >= 0; --v) {
    for (int u = 0; u < width; ++u) {
      for (int c = 0; c < num_channels; ++c) {
        image_ptr[c] =
            static_cast<typename Image<kPixelType>::T>(image.at(u, v)[c]);
      }
      image_ptr += num_scalar_components;
    }
  }

  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(filepath.c_str());
  writer->SetInputData(vtk_image.GetPointer());
  writer->Write();
};

template <PixelType kPixelType>
void SaveLabelToFile(const string& filepath, const Image<kPixelType>& image) {
  const int width = image.width();
  const int height = image.height();
  const int num_channels = 4;

  vtkNew<vtkImageData> vtk_image;
  vtk_image->SetDimensions(width, height, 1);
  vtk_image->AllocateScalars(VTK_UNSIGNED_CHAR, num_channels);

  auto image_ptr =
      reinterpret_cast<unsigned char*>(vtk_image->GetScalarPointer());
  const int num_scalar_components = vtk_image->GetNumberOfScalarComponents();

  for (int v = height - 1; v >= 0; --v) {
    for (int u = 0; u < width; ++u) {
      if (image.at(u, v)[0] == Label::kNoBody ||
          image.at(u, v)[0] == Label::kFlatTerrain) {
        image_ptr[0] = 255;
        image_ptr[1] = 255;
        image_ptr[2] = 255;
        image_ptr[3] = 255;
      } else {
        auto p = static_cast<unsigned char>(image.at(u, v)[0]);
        image_ptr[0] = p;
        image_ptr[1] = p;
        image_ptr[2] = p;
        image_ptr[3] = 255;
      }
      image_ptr += num_scalar_components;
    }
  }

  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(filepath.c_str());
  writer->SetInputData(vtk_image.GetPointer());
  writer->Write();
};

bool ValidateSdf(const char* flagname, const std::string& filename) {
  if (filename.substr(filename.find_last_of(".") + 1) == "sdf") {
    return true;
  }
  cout << "Invalid filename for --" << flagname << ": " << filename << endl;
  return false;
}

bool ValidateDir(const char* flagname, const std::string& dir) {
  if (dir.empty()) {
    cout << "Invalid directory for --" << flagname << ": " << dir << endl;
    return false;
  }
  return true;
}

DEFINE_bool(show_window, true,
            "If true, RgbdCamera opens windows for displaying rendering "
            "context.");
DEFINE_double(duration, 1., "Total duration of the simulation in secondes.");
DEFINE_int32(num, 0, "Start ID of SDF.");
DEFINE_string(sdf_dir, "",
              "The full path of directory where SDFs are located.");
DEFINE_string(sdf_fixed, "sphere.sdf",
              "The filename for a SDF that contains fixed base objects.");
DEFINE_string(sdf_floating, "box.sdf",
              "The filename for a SDF that contains floating base objects.");
DEFINE_validator(sdf_dir, &ValidateDir);
DEFINE_validator(sdf_fixed, &ValidateSdf);
DEFINE_validator(sdf_floating, &ValidateSdf);

constexpr double kCameraUpdatePeriod{0.01};

struct CameraConfig {
  Eigen::Vector3d pos;
  Eigen::Vector3d rpy;
  double fov_y{};
  double depth_range_near{};
  double depth_range_far{};
};

}  // anonymous namespace

bool CheckEmpty(Image<PixelType::kLabel16I>& label) {
  const int width = label.width();
  const int height = label.height();

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      if (label.at(u, v)[0] != Label::kFlatTerrain &&
          label.at(u, v)[0] != 1) {
        return false;  // No, it's not empty.
      }
    }
  }

  return true; // Yes, it's empty.
}

void Generate(int num) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf_dir + "/" + FLAGS_sdf_fixed, kFixed, tree.get());

  std::string n = std::to_string(num);
  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf_dir + "/silverware" + n + ".sdf",
      kQuaternion, tree.get());

  drake::multibody::AddFlatTerrainToWorld(tree.get());

  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree));
  plant->set_name("rigid_body_plant");

  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(1e8)  // Pa
      .set_dissipation(1)  // s/m
      .set_friction(0.9, 0.5);
  plant->set_default_compliant_material(default_material);

  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = 2e-4;  // m
  model_parameters.v_stiction_tolerance = 0.01;  // m/s
  plant->set_contact_model_parameters(model_parameters);

  // Adds an RgbdCamera at a fixed pose.
  CameraConfig config;
  config.pos = Eigen::Vector3d(-0.4, 0., 1.);
  config.rpy = Eigen::Vector3d(0., M_PI_2 * 0.8, 0.);
  config.fov_y = M_PI_4;
  config.depth_range_near = 0.5;
  config.depth_range_far = 5.;

  auto rgbd_camera =
      builder.AddSystem<RgbdCameraDiscrete>(
          std::make_unique<RgbdCamera>(
              "rgbd_camera", plant->get_rigid_body_tree(),
              config.pos, config.rpy,
              config.depth_range_near, config.depth_range_far,
              config.fov_y, FLAGS_show_window),
      kCameraUpdatePeriod);

  ::drake::lcm::DrakeLcm lcm;
  auto drake_viz = builder.template AddSystem<DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);
  drake_viz->set_publish_period(kCameraUpdatePeriod);

  builder.Connect(
      plant->get_output_port(0),
      rgbd_camera->state_input_port());

  builder.Connect(
      plant->get_output_port(0),
      drake_viz->get_input_port(0));

  builder.ExportOutput(rgbd_camera->color_image_output_port());
  builder.ExportOutput(rgbd_camera->label_image_output_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput(*context);

  auto simulator = std::make_unique<systems::Simulator<double>>(
      *diagram, std::move(context));

  simulator->set_publish_at_initialization(true);
  simulator->set_publish_every_time_step(false);
  simulator->Initialize();

  simulator->reset_integrator<RungeKutta2Integrator<double>>(
      *diagram, 0.0001, &simulator->get_mutable_context());
  simulator->get_mutable_integrator()->set_maximum_step_size(0.001);
  simulator->get_mutable_integrator()->set_fixed_step_mode(true);

  simulator->StepTo(FLAGS_duration);
  diagram->CalcOutput(simulator->get_context(), output.get());

  auto sys_label =
      output->GetMutableData(1)->GetMutableValue<ImageLabel16I>();

  if (!CheckEmpty(sys_label)) {
    SaveLabelToFile<PixelType::kLabel16I>(
        std::string("/home/kunimatsu/images/label") + n + ".png", sys_label);

    auto sys_rgb =
        output->GetMutableData(0)->GetMutableValue<ImageRgba8U>();
    SaveToFile<PixelType::kRgba8U>(
        std::string("/home/kunimatsu/images/rgb") + n + ".png", sys_rgb);
  }
}


int main() {
  drake::unused(sdf_dir_validator_registered);
  drake::unused(sdf_fixed_validator_registered);
  drake::unused(sdf_floating_validator_registered);

  for (int i = FLAGS_num; i < 10000; ++i) {
    std::cout << "Simulating No. " << i << "." << std::endl;
    Generate(i);
  }

  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::sensors::main();
}
