#include <limits.h>
#include <locale.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <iostream>

#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "drake/systems/sensors/godot_renderer/godot_scene.h"

using namespace godotvis;

std::string user_root = "/home/sean/";
std::string godot_root = user_root + "code/";
std::string path = godot_root + "godot-demo-projects/3d/material_testers/";
std::string gltf_path = godot_root + "glTF-Sample-Models/2.0/DamagedHelmet/glTF/DamagedHelmet.gltf";
std::string save_path = user_root + "Downloads/godot/";

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

  int cube_id = scene.AddCubeInstance(1., 1., 1.);

  int sphere_id = scene.AddSphereInstance(0.5);
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1.0, 0., 0.);
  scene.SetInstancePose(sphere_id, pose);

  int cylinder_id = scene.AddCylinderInstance(0.5, 2.0);
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-1.0, 0., 0.);
  pose.rotate(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX()));
  scene.SetInstancePose(cylinder_id, pose);

  int plane_id = scene.AddPlaneInstance(5.0, 5.0);
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
  test_GLTF();
  return 0; //os_.get_exit_code();
}
