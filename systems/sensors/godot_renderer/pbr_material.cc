#include "systems/sensors/godot_renderer/pbr_material.h"

namespace drake {
namespace systems {
namespace sensors {

PbrMaterial::PbrMaterial() = default;

PbrMaterial::PbrMaterial(const Eigen::Vector3d& albedo, double metallic,
                         double roughness)
    : albedo_(albedo), metallic_(metallic), roughness_(roughness) {}

}  // namespace sensors
}  // namespace systems
}  // namespace drake