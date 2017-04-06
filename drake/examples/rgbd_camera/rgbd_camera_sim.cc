// Copyright 2016 Toyota Research Institute.  All rights reserved.

// NOLINTNEXTLINE(build/c++11)
#define OPENCV

#include <iostream>

#ifdef OPENCV
#include <opencv2/opencv.hpp>
#endif

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/systems/sensors/image.h"

using std::make_unique;
using std::move;
using std::unique_ptr;
using Eigen::Vector3d;

namespace drake {
using multibody::joints::kFixed;
using multibody::joints::kQuaternion;
using systems::ConstantVectorSource;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::sensors::RgbdCamera;

namespace {

DEFINE_double(duration, 5., "Total duration of the simulation in seconds.");
// TODO(kunimatsu.hashimoto) Add support for image of arbitrary size.
const uint32_t kImageWidth = 640;
const uint32_t kImageHeight = 480;
const uint32_t kNumPixels = kImageWidth * kImageHeight;
const double kFrameRate = 30.;
const double kVerticalFov = M_PI_4;

// A demo of an RgbdCamera rendering simulator.
class RenderingSim : public systems::Diagram<double> {
 public:
  explicit RenderingSim(const std::string& filename) {
    this->set_name("RenderingSim");

    const size_t last_dot = filename.find_last_of(".");
    if (last_dot == std::string::npos) {
      DRAKE_DEMAND(false);
    }
    std::string ext = filename.substr(last_dot, std::string::npos);

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto tree = make_unique<RigidBodyTree<double>>();
    if (ext == ".sdf") {
      drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
          filename, kQuaternion, tree.get());
    } else if (ext == ".urdf") {

      drake::parsers::PackageMap package_map;
      package_map.Add(
          "hsrb_description",
          "/home/kunimatsu/ws_hsrb/src/hsrb_description/");
      package_map.Add(
          "Atlas",
          "/home/kunimatsu/work/kuni-drake/drake/drake/examples/Atlas/");
      drake::parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages(
          filename, package_map, kFixed, /*kQuaternion,*/ nullptr, tree.get());
    } else {
      DRAKE_DEMAND(false);
    }

    drake::multibody::AddFlatTerrainToWorld(tree.get());
    systems::DiagramBuilder<double> builder;

    // Instantiates a RigidBodyPlant from the MBD model of the world.
    plant_ = builder.AddSystem<RigidBodyPlant<double>>(move(tree));
    plant_->set_normal_contact_parameters(3000, 10);
    plant_->set_friction_contact_parameters(0.9, 0.5, 0.01);

    for (int instance_id =
             RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;
         instance_id < plant_->get_num_model_instances(); ++instance_id) {
      if (plant_->model_instance_has_actuators(instance_id)) {
        const int input_port_index =
            plant_->model_instance_actuator_command_input_port(instance_id)
            .get_index();
        const systems::InputPortDescriptor<double>& input_port =
            plant_->get_input_port(input_port_index);
        VectorX<double> constant_value(input_port.size());
        constant_value.setZero();

        // Cascades the constant source to the model instance within the plant
        // and visualizer diagram. This effectively results in the model
        // instance being uncontrolled, i.e., passive.
        systems::ConstantVectorSource<double>* constant_vector_source =
            builder.template AddSystem<systems::ConstantVectorSource<double>>(
                constant_value);
        constant_vector_source->set_name("ConstantVectorZeroSource");
        builder.Connect(constant_vector_source->get_output_port(), input_port);
      }
    }

    // Creates and adds LCM publisher for visualization.
    viz_publisher_ = builder.AddSystem<DrakeVisualizer>(
        plant_->get_rigid_body_tree(), &lcm_);

    rgbd_camera_ = builder.AddSystem<RgbdCamera>(
        "rgbd_camera", plant_->get_rigid_body_tree(),
        Vector3d(1., 0, 1.5), Vector3d(0., M_PI_4, M_PI),
        kVerticalFov, true);

    // builder.Connect(constant_vector_source->get_output_port(),
    //                 plant_->get_input_port(0));

    // Connects to publisher for visualization.
    builder.Connect(plant_->get_output_port(0),
                    viz_publisher_->get_input_port(0));

    builder.Connect(plant_->get_output_port(0),
                    rgbd_camera_->get_input_port(0));

    builder.ExportOutput(rgbd_camera_->color_image_output_port());
    builder.ExportOutput(rgbd_camera_->depth_image_output_port());
    builder.ExportOutput(rgbd_camera_->camera_base_pose_output_port());

    for (int i = 0; i < plant_->get_num_output_ports(); ++i) {
      builder.ExportOutput(plant_->get_output_port(i));
    }

    builder.BuildInto(this);
  }

  Eigen::Isometry3d X_BD() {
    return rgbd_camera_->depth_camera_optical_pose();
  }

 private:
  RigidBodyPlant<double>* plant_;
  DrakeVisualizer* viz_publisher_;
  lcm::DrakeLcm lcm_;
  RgbdCamera* rgbd_camera_;
  ConstantVectorSource<double>* const_source_;
};

}  // namespace

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  if (argc != 2) {
    std::cout << "Usage:" << std::endl;
    std::cout << "  $ rgbd_camera_simulation [options] path_to_sdf"
              << std::endl;
    std::cout << "    options:" << std::endl;
    std::cout << "      -duration=(seconds)     the duration for simulation."
              << std::endl;
    return 0;
  }

  RenderingSim diagram(argv[1]);

  unique_ptr<systems::Context<double>> context =
      diagram.CreateDefaultContext();
  unique_ptr<systems::SystemOutput<double>> output =
      diagram.AllocateOutput(*context);
  systems::Simulator<double> simulator(diagram, move(context));

  // systems::Simulator<double> simulator(diagram);
  simulator.Initialize();

  simulator.set_target_realtime_rate(1.);
  // Simulate for the desired duration.
  for (double time = 0.; time < FLAGS_duration; time += .0333) {
    simulator.StepTo(time);
    diagram.CalcOutput(simulator.get_context(), output.get());
  }

  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  auto color_image =
      mutable_data->GetMutableValue<drake::systems::sensors::Image<uint8_t>>();

  systems::AbstractValue* mutable_data_d = output->GetMutableData(1);
  auto depth_image =
      mutable_data_d->GetMutableValue<drake::systems::sensors::Image<float>>();

  auto X_WC = dynamic_cast<drake::systems::rendering::PoseVector<double>*>(
      output->GetMutableVectorData(2));
  auto X_WD = X_WC->get_isometry() * diagram.X_BD();

  std::cout << X_WD.matrix() << std::endl;

#ifdef OPENCV
  cv::Mat cv_color(kImageHeight, kImageWidth, CV_8UC4, cv::Scalar(0, 0, 0, 255));
  cv::Mat cv_depth(kImageHeight, kImageWidth, CV_16UC1, cv::Scalar(0));

  for (uint32_t r = 0; r < kImageHeight; ++r) {
    for (uint32_t c = 0; c < kImageWidth; ++c) {
      cv_color.at<cv::Vec4b>(r, c)[0] = color_image.at(c, r)[0];
      cv_color.at<cv::Vec4b>(r, c)[1] = color_image.at(c, r)[1];
      cv_color.at<cv::Vec4b>(r, c)[2] = color_image.at(c, r)[2];
      cv_color.at<cv::Vec4b>(r, c)[3] = color_image.at(c, r)[3];
      cv_depth.at<uint16_t>(r, c) = static_cast<uint16_t>(
          depth_image.at(c, r)[0] * 1000.f);
    }
  }

  cv::imwrite("color.png", cv_color);
  cv::imwrite("depth.png", cv_depth);
#endif

  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::main(argc, argv);
}
