#include <iomanip>
#include <iostream>
#include <limits.h>
#include <locale.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

#include <gflags/gflags.h>

#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "drake/systems/sensors/godot_renderer/godot_scene.h"

using namespace godotvis;

std::string user_root = "/home/sean/";
std::string godot_root = user_root + "code/";
std::string path = godot_root + "godot-demo-projects/3d/material_testers/";
std::string gltf_path = godot_root + "glTF-Sample-Models/2.0/DamagedHelmet/glTF/DamagedHelmet.gltf";
std::string save_path = user_root + "Downloads/godot/";

DEFINE_double(x, 0, "light x-position");
DEFINE_double(y, 0, "light y-position");
DEFINE_double(z, 0.1, "light z-position");

// I'm using this to confirm that I can illuminate a ground plane.
void test_plane() {
  using std::to_string;
  GodotRenderer renderer(640, 480);
  GodotScene scene;
  scene.Initialize();
  scene.SetupEnvironment(path + "lobby.hdr");
  scene.AddOmniLight(FLAGS_x, FLAGS_y, FLAGS_z);
  scene.AddCamera(65.0, 0.1, 100.0);
  const double size = 2;
  Color plane_color{0.0, 1.0, 0.0};
  scene.AddPlaneInstance(size, size, plane_color, plane_color);
  Color sphere_color{1.0, 1.0, 0.0};
  scene.AddSphereInstance(0.125, sphere_color, sphere_color);
  const int SAMPLE_COUNT = 20;
  const double start_angle = -M_PI_2;
  const double end_angle = M_PI_2;
  const double d_theta = (end_angle - start_angle) / (SAMPLE_COUNT - 1);
  // CAMERA defaults to looking in the negative-z direction. We'll move it one
  // meter away from the origin and rotate around the world' vertical axis in
  // a semi-circular path.
  scene.ApplyLabelShader();
  Eigen::Isometry3d camera_pose;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    double theta = start_angle + i * d_theta;
    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);
    camera_pose.translation() << s_theta, 0, c_theta;
    camera_pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix();
    scene.SetCameraPose(camera_pose);
    scene.FlushTransformNotifications();
    renderer.Draw();
    Ref<Image> image = scene.Capture();
    image->save_png((save_path + "plane" + to_string(i) + ".png").c_str());
  }
  scene.FlushTransformNotifications();
  renderer.Draw();
  Ref<Image> image = scene.Capture();
  image->save_png((save_path + "plane.png").c_str());
  scene.Finish();
  image.unref();
}

// Tests the loading of a gltf file and its rendering.
void test_GLTF() {
  GodotRenderer renderer(1280, 960);
  GodotScene scene;
  scene.Initialize();
  scene.SetupEnvironment(path + "lobby.hdr");
  scene.AddCamera(65.0, 0.1, 100.0);
  Eigen::Isometry3d camera_pose{Eigen::Isometry3d::Identity()};
  camera_pose.translation() = Eigen::Vector3d(0., 0., 3.);
  scene.SetCameraPose(camera_pose);
  scene.ImportGltf(gltf_path);
  scene.FlushTransformNotifications();
  scene.set_viewport_size(1280, 960);
  renderer.Draw();
  Ref<Image> image = scene.Capture();
  image->save_png((save_path + "gltf.png").c_str());
  scene.Finish();
  image.unref();
}

void RenderFork() {
  GodotRenderer renderer(1280, 960);
  GodotScene scene;
  scene.Initialize();
  scene.SetupEnvironment(path + "lobby.hdr");
  scene.AddCamera(65.0, 0.1, 100.0);
  Eigen::Isometry3d camera_pose{Eigen::Isometry3d::Identity()};
  camera_pose.translation() = Eigen::Vector3d(0., 0., 3.);
  scene.SetCameraPose(camera_pose);
  scene.set_viewport_size(1280, 960);

  Color mesh_color{1.0, 1.0, 1.0};
  int id = scene.AddMeshInstance("/home/sean/code/godot_projects/fork.mesh", mesh_color, mesh_color);
  Eigen::Isometry3d pose{Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d{1, 1, 1})};
  scene.SetInstanceScale(id, 0.25, 0.25, 0.25);

  const double theta_0 = 0;
  const double theta_N = 2 * M_PI;
  const int N = 120;
  const double d_theta = (theta_N - theta_0) / (N - 1);
  Ref<Image> image;
  for (int i = 0; i < N; ++i) {
    pose.linear() = Eigen::AngleAxisd(i * d_theta, Eigen::Vector3d{1, 1, 1}).matrix();
    pose.translation() << 0, 0, -3;
    scene.SetInstancePose(id, pose);
    scene.FlushTransformNotifications();
    renderer.Draw();
    image = scene.Capture();
    std::stringstream ss;
    ss << save_path << "fork_";
    ss << std::setw(4) << std::setfill('0') << i << ".png";
    image->save_png(ss.str().c_str());
  }

  scene.Finish();
  image.unref();
}

void test_primitives() {
  GodotRenderer renderer(640, 480);

  GodotScene scene;
  scene.Initialize();
  scene.SetupEnvironment(path + "night.hdr");
  scene.AddCamera(65.0, 0.1, 100.0);

  Eigen::Isometry3d camera_pose{Eigen::Isometry3d::Identity()};
  camera_pose.translation() = Eigen::Vector3d(0., 0., 15.);
  scene.SetCameraPose(camera_pose);

  //int id = scene.AddMeshInstance(path + "godot_ball.mesh");
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  //scene.SetInstancePose(id, pose);
  //scene.SetInstanceScale(id, 0.8, 0.8, 0.8);

  //Transform godot_T = ConvertToGodotTransform(pose);
  //Transform expected_T{Basis{1., 0., 0., 0., 1., 0., 0., 0., 1.},
                       //Vector3{0., 2.5, 15.0}};
  //if (godot_T != expected_T)
    //std::cout << "FAIL!" << std::endl;
  //else
    //std::cout << "Pass!" << std::endl;

  Color cube_color{1, 0, 0};
  int cube_id = scene.AddCubeInstance(1., 1., 1., cube_color, cube_color);

  Color sphere_color{0, 1, 0};
  int sphere_id = scene.AddSphereInstance(0.5, sphere_color, sphere_color);
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1.0, 0., 0.);
  scene.SetInstancePose(sphere_id, pose);

  Color cylinder_color{0, 0, 1};
  int cylinder_id = scene.AddCylinderInstance(0.5, 2.0, cylinder_color,
                                              cylinder_color);
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-1.0, 0., 0.);
  pose.rotate(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
  scene.SetInstancePose(cylinder_id, pose);

  Color plane_color{0, 1, 1};
  int plane_id = scene.AddPlaneInstance(5.0, 5.0, plane_color, plane_color);
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-1.0, 0., 0.);
  pose.rotate(Eigen::AngleAxisd(M_PI/3.0, Eigen::Vector3d::UnitY()));
  scene.SetInstancePose(plane_id, pose);

  // Need this before drawing to flush all transform changes to the visual server
  scene.FlushTransformNotifications();

  scene.set_viewport_size(1280, 960);
  renderer.Draw();
  // TODO: Why can't I use Ref<Image> variable?
  Ref<Image> image = scene.Capture();
  std::cout << "Image format: " << image->get_format() << std::endl;
  std::cout << "Expected format: " << Image::FORMAT_RGBA8 << std::endl;
  image->save_png((save_path + "rgb1.png").c_str());

  scene.ApplyDepthShader();
  renderer.Draw();
  image = scene.Capture();
  std::cout << "Depth Image format: " << image->get_format() << std::endl;
  std::cout << "Depth Expected format: " << Image::FORMAT_RGBA8 << std::endl;
  image->save_png((save_path + "depth1.png").c_str());

  scene.set_viewport_size(320, 240);
  scene.SetInstanceColor(cube_id, 1., 1., 1.);
  scene.ApplyMaterialShader();
  renderer.Draw();
  image = scene.Capture();
  image->save_png((save_path + "rgb2.png").c_str());

  scene.ApplyDepthShader();
  renderer.Draw();
  image = scene.Capture();
  image->save_png((save_path + "depth2.png").c_str());

  scene.set_viewport_size(640, 480);
  scene.SetInstanceColor(cylinder_id, 1., 1., 1.);
  scene.SetInstanceColor(cube_id, 1., 0., 0.);
  scene.ApplyMaterialShader();
  renderer.Draw();
  scene.Capture()->save_png((save_path + "rgb3.png").c_str());

  scene.ApplyDepthShader();
  renderer.Draw();
  scene.Capture()->save_png((save_path + "depth3.png").c_str());

  scene.Finish();
  image.unref();
}

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  test_plane();
//  RenderFork();
//  test_GLTF();
  return 0; //os_.get_exit_code();
}
