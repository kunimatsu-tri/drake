#pragma once

// Define this macro so glad.h will be included in shader_gles3.h
#define GLES3_INCLUDE_H "glad/glad.h"

#include "core/error_list.h"
#include "core/register_core_types.h"
#include "drivers/gles3/rasterizer_gles3.h"
#include "drivers/register_driver_types.h"
#include "drivers/unix/dir_access_unix.h"
#include "drivers/unix/file_access_unix.h"
#include "os/file_access.h"
#include "os/thread_dummy.h"
#include "os_dummy.h"
#include "project_settings.h"
#include "scene/main/scene_tree.h"
#include "scene/register_scene_types.h"
#include "servers/register_server_types.h"
#include "servers/visual/visual_server_raster.h"

// TL;DR: GLFW has to be included here, after rasterizer_gles3.
// glfw needs to be included *after* glad.h, which is
// included by shader_gles3.h in rasterizer.gles3.h
#include <GLFW/glfw3.h>

namespace godotvis {

class GodotRenderer {
 public:
  GodotRenderer(int window_width, int window_height)
      : window_width_(window_width),
        window_height_(window_height),
        os_(window_width, window_height) {
          Initialize();
        }

  ~GodotRenderer() {
    CleanUp();
  }

  void Initialize();
  void Draw();
  void MainLoop();
  void CleanUp();

  const GLFWwindow*  get_glfw_window() const { return window_; }
  int width() const { return window_width_; }
  int height() const { return window_height_; }

 private:
  // TODO: modify this for render_to_texture
  Error InitGLContext();
  void InitGodot();
  void CleanUpGodot();
  void CleanUpGL();

  int window_width_, window_height_;
  OS_Dummy os_;
  GLFWwindow* window_ = nullptr;
  VisualServer* visual_server_ = nullptr;
  ARVRServer* arvr_server_ = nullptr;
  ProjectSettings* globals_ = nullptr;
  Physics2DServer* physics_2d_server_ =
      nullptr;  //!< Unfortunately this is needed in Viewport's ctor
  PhysicsServer* physics_server_ =
      nullptr;  //!< Unfortunately this is needed in Viewport's ctor
};

}  // namespace godotvis
