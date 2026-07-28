#include "dial/reference.hpp"

#include <algorithm>
#include <cmath>

namespace dial::reference {
namespace {

long double damping(long double velocity,
                    const RigidReferenceParameters& p) noexcept {
  return -p.linearDamping * velocity -
         p.quadraticDamping * std::abs(velocity) * velocity -
         p.coulombFriction *
             std::tanh(velocity /
                       std::max(p.frictionEpsilon, 1.0e-12L));
}

struct Derivative {
  long double angle{};
  long double velocity{};
};

Derivative derivative(const RigidReferenceState& state,
                      const RigidReferenceParameters& p,
                      const RigidReferenceInput& input) noexcept {
  const long double tangentX = std::cos(state.angle);
  const long double tangentY = std::sin(state.angle);
  const long double field =
      p.firstMoment *
      (tangentX * input.effectiveAccelX +
       tangentY * input.effectiveAccelY);
  const long double axisX = std::sin(state.angle);
  const long double axisY = -std::cos(state.angle);
  const long double omegaDotAxis =
      input.bodyOmegaX * axisX + input.bodyOmegaY * axisY;
  const long double omegaDotTangent =
      input.bodyOmegaX * tangentX + input.bodyOmegaY * tangentY;
  const long double centrifugal =
      -p.inertia * omegaDotAxis * omegaDotTangent;
  return {state.angularVelocity,
          (field + centrifugal + input.relativeTorque +
           damping(state.angularVelocity, p)) /
              p.inertia};
}

RigidReferenceState plus(const RigidReferenceState& state,
                         const Derivative& derivativeValue,
                         long double scale) noexcept {
  return {state.angle + derivativeValue.angle * scale,
          state.angularVelocity + derivativeValue.velocity * scale};
}

}  // namespace

long double rigidDerivativeTorque(
    long double angle, const RigidReferenceParameters& parameters,
    const RigidReferenceInput& input) noexcept {
  const long double tangentX = std::cos(angle);
  const long double tangentY = std::sin(angle);
  const long double axisX = std::sin(angle);
  const long double axisY = -std::cos(angle);
  return parameters.firstMoment *
             (tangentX * input.effectiveAccelX +
              tangentY * input.effectiveAccelY) -
         parameters.inertia *
             (input.bodyOmegaX * axisX +
              input.bodyOmegaY * axisY) *
             (input.bodyOmegaX * tangentX +
              input.bodyOmegaY * tangentY);
}

RigidReferenceState integrateRigidRk4(
    RigidReferenceState state,
    const RigidReferenceParameters& parameters,
    const RigidReferenceInput& input, long double duration,
    long double maxStep) noexcept {
  const std::uint64_t steps = static_cast<std::uint64_t>(
      std::max(1.0L, std::ceil(duration / maxStep)));
  const long double step = duration / static_cast<long double>(steps);
  for (std::uint64_t i = 0; i < steps; ++i) {
    const Derivative k1 = derivative(state, parameters, input);
    const Derivative k2 =
        derivative(plus(state, k1, step * 0.5L), parameters, input);
    const Derivative k3 =
        derivative(plus(state, k2, step * 0.5L), parameters, input);
    const Derivative k4 =
        derivative(plus(state, k3, step), parameters, input);
    state.angle +=
        step * (k1.angle + 2.0L * k2.angle + 2.0L * k3.angle +
                k4.angle) /
        6.0L;
    state.angularVelocity +=
        step * (k1.velocity + 2.0L * k2.velocity +
                2.0L * k3.velocity + k4.velocity) /
        6.0L;
  }
  return state;
}

long double FlexibleMassMatrix::determinant() const noexcept {
  return m11 * m22 - m12 * m12;
}

FlexibleMassMatrix flexibleMassMatrix(
    long double beta, const FlexibleDefinition& f) noexcept {
  const long double m1 = f.m1Kg;
  const long double m2 = f.m2Kg;
  const long double d1 = f.d1M;
  const long double l1 = f.l1M;
  const long double l2 = f.l2M;
  const long double j1 = f.j1KgM2;
  const long double j2 = f.j2KgM2;
  const long double cosine = std::cos(beta);
  return {
      j1 + m1 * l1 * l1 + j2 +
          m2 * (d1 * d1 + l2 * l2 + 2.0L * d1 * l2 * cosine),
      j2 + m2 * (l2 * l2 + d1 * l2 * cosine),
      j2 + m2 * l2 * l2};
}

std::array<long double, 2> solveFlexibleAcceleration(
    long double theta, long double beta, long double thetaDot,
    long double betaDot, const FlexibleDefinition& f,
    long double effectiveAccelX, long double effectiveAccelY,
    long double externalRootTorque,
    long double externalBendTorque) noexcept {
  const long double rootTx = std::cos(theta);
  const long double rootTy = std::sin(theta);
  const long double tipTx = std::cos(theta + beta);
  const long double tipTy = std::sin(theta + beta);
  const long double qField0 =
      (static_cast<long double>(f.m1Kg) * f.l1M +
       static_cast<long double>(f.m2Kg) * f.d1M) *
          (rootTx * effectiveAccelX + rootTy * effectiveAccelY) +
      static_cast<long double>(f.m2Kg) * f.l2M *
          (tipTx * effectiveAccelX + tipTy * effectiveAccelY);
  const long double qField1 =
      static_cast<long double>(f.m2Kg) * f.l2M *
      (tipTx * effectiveAccelX + tipTy * effectiveAccelY);
  const long double b =
      static_cast<long double>(f.m2Kg) * f.d1M * f.l2M;
  const long double c0 =
      -b * std::sin(beta) *
      (2.0L * thetaDot * betaDot + betaDot * betaDot);
  const long double c1 =
      b * std::sin(beta) * thetaDot * thetaDot;
  const long double bend =
      -static_cast<long double>(f.bendStiffnessNmRad) * beta -
      static_cast<long double>(f.bendDampingNmSecRad) * betaDot;
  const std::array<long double, 2> rhs{
      qField0 + externalRootTorque - c0,
      qField1 + externalBendTorque + bend - c1};
  const FlexibleMassMatrix mass = flexibleMassMatrix(beta, f);
  const long double inverse = 1.0L / mass.determinant();
  return {(mass.m22 * rhs[0] - mass.m12 * rhs[1]) * inverse,
          (-mass.m12 * rhs[0] + mass.m11 * rhs[1]) * inverse};
}

}  // namespace dial::reference

