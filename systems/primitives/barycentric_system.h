#pragma once

#include <memory>
#include <utility>

#include "drake/math/barycentric.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {

// TODO(russt): Generalize these systems (and the filename) to more general
// function approximators if/when that set of tools comes online (see
// corresponding TODO in drake/math/barycentric.h).

/// A (stateless) vector system implemented as a multi-linear (barycentric)
/// interpolation on a mesh over the inputs.
///
/// This has many potential uses, including representing the policies that are
/// generated by numerical control design methods like approximate dynamic
/// programming.
///
/// @see math::BarycentricMesh
template <typename T>
class BarycentricMeshSystem : public VectorSystem<T> {
  // TODO(russt): Could generalize this to systems with state.
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BarycentricMeshSystem);

  /// Constructs the system from a mesh and the associated mesh values.
  /// @p output_values is a matrix with each column representing the value to
  /// output if the input is the associated mesh point.  It must have the same
  /// number of columns as mesh.get_num_mesh_points();
  /// mesh.MeshValuesFrom(function) is one useful tool for creating it.
  ///
  BarycentricMeshSystem(math::BarycentricMesh<T> mesh,
                        const Eigen::Ref<const MatrixX<T>>& output_values)
      : VectorSystem<T>(mesh.get_input_size(), output_values.rows()),
        mesh_(std::move(mesh)),
        output_values_(output_values) {
    DRAKE_DEMAND(output_values_.rows() > 0);
    DRAKE_DEMAND(output_values_.cols() == mesh_.get_num_mesh_points());
  }

 protected:
  /// Evaluates the BarycentricMesh at the input and writes it to the output.
  virtual void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const {
    unused(context, state);
    mesh_.Eval(output_values_, input, output);
  }

 private:
  const math::BarycentricMesh<T> mesh_;
  const MatrixX<T> output_values_;
};

}  // namespace systems
}  // namespace drake
