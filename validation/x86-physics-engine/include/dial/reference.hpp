#pragma once

#include "dial/physics.hpp"

namespace dial::reference {

struct RigidReferenceState {
  long double angle{};
  long double angularVelocity{};
};

struct RigidReferenceParameters {
  long double inertia{};
  long double firstMoment{};
  long double linearDamping{};
  long double quadraticDamping{};
  long double coulombFriction{};
  long double frictionEpsilon{};
};

struct RigidReferenceInput {
  long double effectiveAccelX{};
  long double effectiveAccelY{};
  long double bodyOmegaX{};
  long double bodyOmegaY{};
  long double bodyOmegaZ{};
  long double relativeTorque{};
};

long double rigidDerivativeTorque(long double angle,
                                  const RigidReferenceParameters& parameters,
                                  const RigidReferenceInput& input) noexcept;

RigidReferenceState integrateRigidRk4(
    RigidReferenceState state,
    const RigidReferenceParameters& parameters,
    const RigidReferenceInput& input,
    long double duration,
    long double maxStep = 1.0e-5L) noexcept;

struct FlexibleMassMatrix {
  long double m11{};
  long double m12{};
  long double m22{};
  [[nodiscard]] long double determinant() const noexcept;
};

FlexibleMassMatrix flexibleMassMatrix(long double beta,
                                      const FlexibleDefinition& definition) noexcept;

std::array<long double, 2> solveFlexibleAcceleration(
    long double theta, long double beta,
    long double thetaDot, long double betaDot,
    const FlexibleDefinition& definition,
    long double effectiveAccelX, long double effectiveAccelY,
    long double externalRootTorque = 0.0L,
    long double externalBendTorque = 0.0L) noexcept;

}  // namespace dial::reference

