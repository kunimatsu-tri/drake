#pragma once

#include "common/eigen_types.h"

namespace drake {
namespace systems {
namespace sensors {

/** Definition of a physically-based rendering (PBR) material. It uses the
 "metallic-roughness" model (see https://www.allegorithmic.com/pbr-guide for a
 discussion). This implementation is a *subset* of the full model as implemented
 by the godot game engine
 (http://docs.godotengine.org/en/3.0/tutorials/3d/spatial_material.html#material-colors-maps-and-channels).
 This version supports the following features:

   - albedo parameter
   - metallic parameter
   - roughness parameter

 The following features are not yet supported:
   - No texture maps (all parameters are defined by object-wide scalar values)
   - emission parameter
   - rim parameter
   - clearcoat parameter
   - anisotropy parameter
   - ambient occlusion parameter
   - depth parameter
   - sub-surface scatter parameter
   - transmission parameter
   - refraction parameter
 */
class PbrMaterial {
 public:
  PbrMaterial();
  PbrMaterial(const Eigen::Vector3d& albedo, double metallic, double roughness);

  void set_albedo(const Eigen::Vector3d& albedo) { albedo_ = albedo; }
  const Eigen::Vector3d& albedo() const { return albedo_; }
  void set_metallic(double metallic) { metallic_ = metallic; }
  double metallic() const { return metallic_; }
  void set_roughness(double roughness) { roughness_ = roughness; }
  double roughness() const { return roughness_; }
 private:
  Eigen::Vector3d albedo_{1.0, 1.0, 1.0};
  double metallic_{0.5};
  double roughness_{0.5};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake