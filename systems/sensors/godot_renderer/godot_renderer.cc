#include "drake/systems/sensors/godot_renderer/godot_renderer.h"
#include "modules/jpg/register_types.h"
#include "modules/hdr/register_types.h"

#include <iostream>

namespace godotvis {

void GodotRenderer::Initialize() {
  InitGLContext();
  InitGodot();
}

void GodotRenderer::Draw() {
  glClear(GL_COLOR_BUFFER_BIT);
  VSG::scene->update_dirty_instances(); // update scene stuff
  VSG::viewport->draw_viewports();
  VSG::scene->render_probes();
}

void GodotRenderer::MainLoop() {
  do {
    Draw();
    // Ref<Image> image = viewport_->get_texture()->get_data();
    // image->save_png("/home/duynguyen/Downloads/texture.png");

    // TODO: modify this for render_to_texture
    glfwSwapBuffers(window_);
    glfwPollEvents();
  } while (glfwGetKey(window_, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
           glfwWindowShouldClose(window_) == 0);
}

void GodotRenderer::CleanUp() {
  CleanUpGodot();
  CleanUpGL();
}

// TODO: modify this for render_to_texture
Error GodotRenderer::InitGLContext() {
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return FAILED;
  }
  glfwWindowHint(GLFW_SAMPLES, 4);               // 4x antialiasing
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,
                 GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE,
                 GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL
  glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

  // Open a window_ and create its OpenGL context
  window_ = glfwCreateWindow(window_width_, window_height_, "", NULL, NULL);
  if (window_ == NULL) {
    fprintf(stderr, "Failed to open GLFW window_.\n");
    glfwTerminate();
    return FAILED;
  }
  glfwMakeContextCurrent(window_); // Initialize GLEW
  glfwSetInputMode(window_, GLFW_STICKY_KEYS, GL_TRUE);
  return OK;
}

void GodotRenderer::InitGodot() {
  // Main::setup()
  RID_OwnerBase::init_rid();

  /// THESE are needed for ObjectDB operations
  ThreadDummy::make_default();
  SemaphoreDummy::make_default();
  MutexDummy::make_default();
  RWLockDummy::make_default();
  IPDummy::make_default();

  // Do we need this? Can we just pass resources directly from Drake?
  FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_RESOURCES);
  FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_USERDATA);
  FileAccess::make_default<FileAccessUnix>(FileAccess::ACCESS_FILESYSTEM);
  ////FileAccessBufferedFA<FileAccessUnix>::make_default();
  ////TODO: Change to DirAccessOSX for osx platform!!!
  DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_RESOURCES);
  DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_USERDATA);
  DirAccess::make_default<DirAccessUnix>(DirAccess::ACCESS_FILESYSTEM);
  // ============================

  ClassDB::init();

  register_core_types();
  register_core_driver_types();

  // Needed for GLOBAL_DEF in VisualServer and Rasterizer
  globals_ = memnew(ProjectSettings);

  // TODO(SeanCurtis-TRI): Get rid of this; it only initializes network
  // settings.
  register_core_settings(); // here globals_ is present
  //////////////////////////////////////
  // Main setup2
  ////////////////////////////////////////////////////////////////

  RasterizerGLES3::register_config();
  RasterizerGLES3::make_current();

  visual_server_ = memnew(VisualServerRaster);
  visual_server_->init();

  //=====================================
  register_server_types();
  Color clear = GLOBAL_DEF("rendering/environment/default_clear_color",
                           Color(0.5, 0.5, 0.5));
  VisualServer::get_singleton()->set_default_clear_color(clear);

  register_scene_types();
  // Register image loaders
  register_jpg_types();
  register_hdr_types();

  arvr_server_ = memnew(ARVRServer); // Needed for VisualServer::draw(), in
                                     // VisualServerViewport::draw_viewports()

  physics_2d_server_ = Physics2DServerManager::new_default_server();
  /// 3D Physics Server
  physics_server_ = PhysicsServerManager::new_default_server();
}

void GodotRenderer::CleanUpGodot() {
  if (physics_2d_server_)
    memdelete(physics_2d_server_);

  if (physics_server_)
    memdelete(physics_server_);

  // CleanUp
  unregister_hdr_types();
  unregister_jpg_types();
  unregister_scene_types();
  unregister_server_types();

  visual_server_->finish();
  memdelete(visual_server_);

  if (arvr_server_)
    memdelete(arvr_server_);

  if (globals_)
    memdelete(globals_);

  unregister_core_driver_types();
  unregister_core_types();
}

void GodotRenderer::CleanUpGL() { glfwTerminate(); }

} // namespace godotvis
