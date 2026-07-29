#include "dial/physics.hpp"

// This translation unit intentionally has no platform, UI, sensor-driver, or
// validation-framework dependency.

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace dial {
namespace {

constexpr float kMinPositive = 1.0e-12F;

float clampValue(float value, float low, float high) noexcept {
  return std::max(low, std::min(value, high));
}

float matrixQuadratic(const Mat3f& matrix, Vec3f left, Vec3f right) noexcept {
  const Vec3f product{
      matrix.v[0][0] * right.x + matrix.v[0][1] * right.y +
          matrix.v[0][2] * right.z,
      matrix.v[1][0] * right.x + matrix.v[1][1] * right.y +
          matrix.v[1][2] * right.z,
      matrix.v[2][0] * right.x + matrix.v[2][1] * right.y +
          matrix.v[2][2] * right.z};
  return dot(left, product);
}

Mat3f add(Mat3f a, const Mat3f& b) noexcept {
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      a.v[row][column] += b.v[row][column];
    }
  }
  return a;
}

bool finite(const Mat3f& matrix) noexcept {
  for (const auto& row : matrix.v) {
    for (float value : row) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
  }
  return true;
}

Vec3f rotateZ(Vec3f vector, float angle) noexcept {
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {cosine * vector.x - sine * vector.y,
          sine * vector.x + cosine * vector.y, vector.z};
}

float dampingTorque(const HandDefinition& hand, float omega) noexcept {
#if DIAL_MUTATION_ID == 5
  return hand.viscousDampingNmSecPerRad * omega +
         hand.quadraticDampingNmSec2PerRad2 * std::abs(omega) * omega +
         hand.coulombFrictionNm *
             std::tanh(omega /
                       std::max(hand.frictionSmoothVelocityRadSec, 1.0e-6F));
#else
  return -hand.viscousDampingNmSecPerRad * omega -
         hand.quadraticDampingNmSec2PerRad2 * std::abs(omega) * omega -
         hand.coulombFrictionNm *
             std::tanh(omega /
                       std::max(hand.frictionSmoothVelocityRadSec, 1.0e-6F));
#endif
}

float limitedPd(float error, float velocityError, float kp, float kd,
                float maximum) noexcept {
  return clampValue(kp * error + kd * velocityError, -maximum, maximum);
}

float intervalSeconds(const BaseSubInterval& interval) noexcept {
  return static_cast<float>(interval.endUs - interval.startUs) * 1.0e-6F;
}

BaseSubInterval coalesce(const BaseInterval& base, std::size_t first,
                         std::size_t count) noexcept {
  BaseSubInterval result = base.quarter[first];
  result.startUs = base.quarter[first].startUs;
  result.endUs = base.quarter[first + count - 1].endUs;
  result.effectiveAccelImpulseMps = {};
  result.omegaOuterProductIntegral = {};
  result.qualityFlags = 0U;
  for (std::size_t i = first; i < first + count; ++i) {
    result.effectiveAccelImpulseMps =
        result.effectiveAccelImpulseMps +
        base.quarter[i].effectiveAccelImpulseMps;
    result.omegaOuterProductIntegral =
        add(result.omegaOuterProductIntegral,
            base.quarter[i].omegaOuterProductIntegral);
    result.qualityFlags |= base.quarter[i].qualityFlags;
  }
  result.omegaBodyAtStartRadSec =
      base.quarter[first].omegaBodyAtStartRadSec;
  result.omegaBodyAtEndRadSec =
      base.quarter[first + count - 1].omegaBodyAtEndRadSec;
  result.gravityBodyAtEndMps2 =
      base.quarter[first + count - 1].gravityBodyAtEndMps2;
  return result;
}

bool validHandIndex(std::int8_t index) noexcept {
  return index >= 0 &&
         static_cast<std::size_t>(index) < static_cast<std::size_t>(kHandCount);
}

struct Mass2 {
  float m11{};
  float m12{};
  float m22{};
  float determinant{};
};

Mass2 flexibleMass(float beta, const FlexibleDefinition& f) noexcept {
  const float cosine = std::cos(beta);
  const float coupling = f.m2Kg * f.d1M * f.l2M;
  Mass2 m{};
  m.m11 = f.j1KgM2 + f.m1Kg * f.l1M * f.l1M + f.j2KgM2 +
          f.m2Kg * (f.d1M * f.d1M + f.l2M * f.l2M +
                    2.0F * f.d1M * f.l2M * cosine);
  m.m12 = f.j2KgM2 +
          f.m2Kg * (f.l2M * f.l2M + f.d1M * f.l2M * cosine);
  m.m22 = f.j2KgM2 + f.m2Kg * f.l2M * f.l2M;
  m.determinant = m.m11 * m.m22 - m.m12 * m.m12;
  (void)coupling;
  return m;
}

std::array<float, 2> inverseMultiply(const Mass2& mass,
                                     std::array<float, 2> vector) noexcept {
  const float inverseDet = 1.0F / mass.determinant;
  return {(mass.m22 * vector[0] - mass.m12 * vector[1]) * inverseDet,
          (-mass.m12 * vector[0] + mass.m11 * vector[1]) * inverseDet};
}

}  // namespace

Vec3f operator+(Vec3f a, Vec3f b) noexcept {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3f operator-(Vec3f a, Vec3f b) noexcept {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3f operator*(Vec3f a, float s) noexcept {
  return {a.x * s, a.y * s, a.z * s};
}

float dot(Vec3f a, Vec3f b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f cross(Vec3f a, Vec3f b) noexcept {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

float norm(Vec3f a) noexcept { return std::sqrt(dot(a, a)); }

bool finite(Vec3f a) noexcept {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

Quatf normalized(Quatf q, bool* corrected) noexcept {
  const float magnitude =
      std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (corrected != nullptr) {
    *corrected = std::abs(magnitude - 1.0F) > 1.0e-6F;
  }
  if (!std::isfinite(magnitude) || magnitude < 1.0e-8F) {
    return {};
  }
  const float inverse = 1.0F / magnitude;
  return {q.w * inverse, q.x * inverse, q.y * inverse, q.z * inverse};
}

Vec3f rotateBodyToWorld(Quatf qWB, Vec3f body) noexcept {
  qWB = normalized(qWB);
  const Vec3f qv{qWB.x, qWB.y, qWB.z};
  const Vec3f twiceCross = cross(qv, body) * 2.0F;
  return body + twiceCross * qWB.w + cross(qv, twiceCross);
}

Vec3f rotateWorldToBody(Quatf qWB, Vec3f world) noexcept {
  qWB = normalized(qWB);
  return rotateBodyToWorld({qWB.w, -qWB.x, -qWB.y, -qWB.z}, world);
}

Vec3f gravityBody(Quatf qWB, float gravity) noexcept {
  return rotateWorldToBody(qWB, {0.0F, 0.0F, -gravity});
}

Vec3f effectiveAccelerationAtCenter(
    Vec3f specificForceAtSensor, Vec3f bodyAngularVelocity,
    Vec3f bodyAngularAcceleration, Vec3f centerToSensorBody) noexcept {
  return specificForceAtSensor * -1.0F +
         cross(bodyAngularAcceleration, centerToSensorBody) +
         cross(bodyAngularVelocity,
               cross(bodyAngularVelocity, centerToSensorBody));
}

Vec3f handAxis(float angle) noexcept {
  return {std::sin(angle), -std::cos(angle), 0.0F};
}

Vec3f handTangent(float angle) noexcept {
  return {std::cos(angle), std::sin(angle), 0.0F};
}

float shortestAngle(float from, float to) noexcept {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double LogicalAngle::value() const noexcept {
  return static_cast<double>(turns) * static_cast<double>(kTwoPi) +
         static_cast<double>(phase);
}

void LogicalAngle::set(double radians) noexcept {
  const double turnsDouble =
      std::floor((radians + static_cast<double>(kPi)) /
                 static_cast<double>(kTwoPi));
  turns = static_cast<std::int64_t>(turnsDouble);
  phase = static_cast<float>(
      radians - turnsDouble * static_cast<double>(kTwoPi));
  if (phase >= kPi) {
    phase -= kTwoPi;
    ++turns;
  }
}

void LogicalAngle::advance(float deltaRadians) noexcept {
  phase += deltaRadians;
  while (phase >= kPi) {
    phase -= kTwoPi;
    ++turns;
  }
  while (phase < -kPi) {
    phase += kTwoPi;
    --turns;
  }
}

EngineConfig makeDefaultConfig() noexcept {
  EngineConfig config{};
  struct BaseHand {
    float mass;
    float length;
    float center;
    float inertia;
    float dampingRatio;
    HandKind kind;
  };
  constexpr std::array<BaseHand, kHandCount> base{{
      {0.0004F, 0.010F, 0.0045F, 4.0e-8F, 0.12F,
       HandKind::Rigid},
      {0.0003F, 0.014F, 0.0065F, 4.0e-8F, 0.12F,
       HandKind::Flexible2D},
      {0.00015F, 0.016F, 0.0075F, 3.0e-8F, 0.10F,
       HandKind::Rigid},
  }};
  for (std::size_t i = 0; i < kHandCount; ++i) {
    HandDefinition hand{};
    hand.kind = base[i].kind;
    hand.massKg = base[i].mass;
    hand.visualLengthM = base[i].length;
    hand.centerOfMassAtZeroM = {0.0F, -base[i].center, 0.0F};
    hand.inertiaPivotKgM2 = base[i].inertia;
    const float gravityStiffness =
        hand.massKg * base[i].center * 9.80665F;
    hand.viscousDampingNmSecPerRad =
        2.0F * base[i].dampingRatio *
        std::sqrt(hand.inertiaPivotKgM2 * gravityStiffness);
    hand.quadraticDampingNmSec2PerRad2 =
        hand.viscousDampingNmSecPerRad * 0.01F;
    hand.coulombFrictionNm = gravityStiffness * 0.002F;
    hand.frictionSmoothVelocityRadSec = 0.02F;
    hand.timeKpNmPerRad = hand.inertiaPivotKgM2 * 8.0F * 8.0F;
    hand.timeKdNmSecPerRad =
        2.0F * 0.85F * hand.inertiaPivotKgM2 * 8.0F;
    hand.maxTimeTorqueNm = hand.timeKpNmPerRad * 4.0F;
    hand.touchKpNmPerRad = hand.inertiaPivotKgM2 * 20.0F * 20.0F;
    hand.touchKdNmSecPerRad =
        2.0F * 0.75F * hand.inertiaPivotKgM2 * 20.0F;
    hand.maxTouchTorqueNm = hand.touchKpNmPerRad;
    hand.maxAngularSpeedRadSec = 80.0F;
    hand.maxRelativeKineticEnergyJ =
        0.5F * hand.inertiaPivotKgM2 * 80.0F * 80.0F;
    config.hands[i] = hand;
  }
  config.hands[1].flex = {0.00018F, 0.00012F, 0.007F, 0.0035F,
                          0.0035F, 6.0e-8F, 2.5e-8F, 3.0e-7F,
                          5.0e-9F, 0.35F, 1.0e-7F};
  return config;
}

float maximumDimensionlessSubstep(const EngineConfig& config) noexcept {
  float maximum = 0.0F;
  const float dt = config.baseDtSec /
                   static_cast<float>(std::max<std::uint8_t>(
                       config.substeps, static_cast<std::uint8_t>(1U)));
  for (const HandDefinition& hand : config.hands) {
    const float firstMoment =
        hand.massKg *
        std::sqrt(hand.centerOfMassAtZeroM.x *
                      hand.centerOfMassAtZeroM.x +
                  hand.centerOfMassAtZeroM.y *
                      hand.centerOfMassAtZeroM.y);
    if (hand.kind == HandKind::Rigid) {
      const float stiffness =
          firstMoment * config.maxEffectiveAccelerationMps2 +
          hand.timeKpNmPerRad + hand.touchKpNmPerRad +
          hand.inertiaPivotKgM2 *
              config.maxBodyAngularVelocityRadSec *
              config.maxBodyAngularVelocityRadSec;
      maximum =
          std::max(maximum,
                   std::sqrt(stiffness / hand.inertiaPivotKgM2) * dt);
      continue;
    }
    const FlexibleDefinition& flex = hand.flex;
    for (int sample = 0; sample <= 32; ++sample) {
      const float beta =
          -kPi + kTwoPi * static_cast<float>(sample) / 32.0F;
      const Mass2 mass = flexibleMass(beta, flex);
      const float fieldRoot =
          (flex.m1Kg * flex.l1M + flex.m2Kg * flex.d1M +
           flex.m2Kg * flex.l2M) *
          config.maxEffectiveAccelerationMps2;
      const float fieldBend =
          flex.m2Kg * flex.l2M *
          config.maxEffectiveAccelerationMps2;
      const float omegaSquared =
          config.maxBodyAngularVelocityRadSec *
          config.maxBodyAngularVelocityRadSec;
      const float k11 =
          fieldRoot + hand.timeKpNmPerRad + hand.touchKpNmPerRad +
          mass.m11 * omegaSquared;
      const float k12 = hand.touchKpNmPerRad;
      const float k22 =
          fieldBend + flex.bendStiffnessNmRad +
          hand.touchKpNmPerRad + mass.m22 * omegaSquared;
      const float invDet = 1.0F / mass.determinant;
      const float a11 = (mass.m22 * k11 - mass.m12 * k12) * invDet;
      const float a12 = (mass.m22 * k12 - mass.m12 * k22) * invDet;
      const float a21 = (-mass.m12 * k11 + mass.m11 * k12) * invDet;
      const float a22 = (-mass.m12 * k12 + mass.m11 * k22) * invDet;
      const float trace = a11 + a22;
      const float determinant = a11 * a22 - a12 * a21;
      const float discriminant =
          std::max(0.0F, trace * trace - 4.0F * determinant);
      const float lambdaMaximum =
          0.5F * (trace + std::sqrt(discriminant));
      maximum =
          std::max(maximum, std::sqrt(std::max(0.0F, lambdaMaximum)) * dt);
    }
  }
  return maximum;
}

PhysicalDialEngine::PhysicalDialEngine() noexcept {
  (void)configure(makeDefaultConfig());
  reset({}, {}, {});
}

bool PhysicalDialEngine::validateConfig(const EngineConfig& config) noexcept {
  if (!std::isfinite(config.baseDtSec) || config.baseDtSec <= 0.0F ||
      (config.substeps != 1U && config.substeps != 2U &&
       config.substeps != 4U)) {
    return false;
  }
  for (const HandDefinition& hand : config.hands) {
    const float radiusSquared =
        hand.centerOfMassAtZeroM.x * hand.centerOfMassAtZeroM.x +
        hand.centerOfMassAtZeroM.y * hand.centerOfMassAtZeroM.y;
    if (!(hand.massKg > 0.0F) || !(hand.visualLengthM > 0.0F) ||
        !(hand.inertiaPivotKgM2 > 0.0F) ||
        hand.inertiaPivotKgM2 + 1.0e-15F <
            hand.massKg * radiusSquared ||
        hand.viscousDampingNmSecPerRad < 0.0F ||
        hand.quadraticDampingNmSec2PerRad2 < 0.0F ||
        hand.coulombFrictionNm < 0.0F ||
        !(hand.frictionSmoothVelocityRadSec > 0.0F) ||
        !(hand.maxAngularSpeedRadSec > 0.0F) ||
        !(hand.maxRelativeKineticEnergyJ > 0.0F) ||
        !finite(hand.centerOfMassAtZeroM)) {
      return false;
    }
    if (hand.kind == HandKind::Flexible2D) {
      const FlexibleDefinition& flex = hand.flex;
      if (!(flex.m1Kg > 0.0F) || !(flex.m2Kg > 0.0F) ||
          !(flex.d1M > 0.0F) || !(flex.l1M > 0.0F) ||
          !(flex.l2M > 0.0F) || !(flex.j1KgM2 > 0.0F) ||
          !(flex.j2KgM2 > 0.0F) ||
          flex.bendStiffnessNmRad < 0.0F ||
          flex.bendDampingNmSecRad < 0.0F) {
        return false;
      }
      for (int sample = 0; sample <= 32; ++sample) {
        const float beta =
            -kPi + kTwoPi * static_cast<float>(sample) / 32.0F;
        const Mass2 mass = flexibleMass(beta, flex);
        if (!(mass.m11 > 0.0F) || !(mass.m22 > 0.0F) ||
            !(mass.determinant > 1.0e-22F)) {
          return false;
        }
      }
    }
  }
  return maximumDimensionlessSubstep(config) <= 0.25F;
}

bool PhysicalDialEngine::configure(const EngineConfig& config) noexcept {
  if (!validateConfig(config)) {
    snapshot_.lastError = ErrorCode::InvalidConfiguration;
    configured_ = false;
    return false;
  }
  config_ = config;
  configured_ = true;
  snapshot_.lastError = ErrorCode::None;
  return true;
}

void PhysicalDialEngine::reset(
    const std::array<double, kHandCount>& angles,
    const std::array<float, kHandCount>& relativeVelocities,
    Vec3f baseOmega) noexcept {
  for (std::size_t i = 0; i < kHandCount; ++i) {
    states_[i] = {};
    states_[i].mode = HandMode::TimeControlled;
    states_[i].rigid.angle.set(angles[i]);
    states_[i].rigid.momentum =
        config_.hands[i].inertiaPivotKgM2 *
        (relativeVelocities[i] + baseOmega.z);
    states_[i].flexible.rootAngle.set(angles[i]);
    states_[i].flexible.qDot[0] = relativeVelocities[i];
  }
  snapshot_ = {};
  globalMode_ = GlobalMode::TimeTracking;
  idleTimeSec_ = 0.0F;
  expectedStartUs_ = 0U;
  publish({});
}

bool PhysicalDialEngine::resetFlexibleState(
    std::size_t hand, double rootAngle, float bendAngle,
    float rootVelocity, float bendVelocity) noexcept {
  if (hand >= kHandCount ||
      config_.hands[hand].kind != HandKind::Flexible2D ||
      !std::isfinite(rootAngle) || !std::isfinite(bendAngle) ||
      !std::isfinite(rootVelocity) || !std::isfinite(bendVelocity)) {
    return false;
  }
  RuntimeHand& state = states_[hand];
  state.flexible.rootAngle.set(rootAngle);
  state.flexible.beta = bendAngle;
  state.flexible.qDot[0] = rootVelocity;
  state.flexible.qDot[1] = bendVelocity;
  state.mode = HandMode::ImuFree;
  state.settled = false;
  state.flags = 0U;
  return true;
}

bool PhysicalDialEngine::validateInterval(const BaseInterval& base) noexcept {
  if (base.endUs <= base.startUs) {
    return false;
  }
  if (expectedStartUs_ != 0U && base.startUs != expectedStartUs_ &&
      (base.qualityFlags & QualityGap) == 0U) {
    snapshot_.lastError = ErrorCode::TimestampDiscontinuity;
    return false;
  }
  std::uint64_t cursor = base.startUs;
  for (const BaseSubInterval& quarter : base.quarter) {
    if (quarter.startUs != cursor || quarter.endUs <= quarter.startUs ||
        !finite(quarter.effectiveAccelImpulseMps) ||
        !finite(quarter.omegaOuterProductIntegral) ||
        !finite(quarter.omegaBodyAtStartRadSec) ||
        !finite(quarter.omegaBodyAtEndRadSec) ||
        !finite(quarter.gravityBodyAtEndMps2)) {
      return false;
    }
    cursor = quarter.endUs;
  }
  return cursor == base.endUs;
}

void PhysicalDialEngine::applyEvents(const StepInput& input,
                                     Vec3f baseOmega) noexcept {
  const std::size_t count = std::min(input.eventCount, kMaxEvents);
  for (std::size_t eventIndex = 0; eventIndex < count; ++eventIndex) {
    const PhysicsEvent& event = input.events[eventIndex];
    if (event.hand >= kHandCount && event.type != EventType::EnterPhysics &&
        event.type != EventType::BeginReturn &&
        event.type != EventType::ForceTimeTracking &&
        event.type != EventType::Recover) {
      continue;
    }
    switch (event.type) {
      case EventType::EnterPhysics:
        globalMode_ = GlobalMode::PhysicsPlay;
        for (RuntimeHand& state : states_) {
          state.animation = {};
          state.mode = HandMode::ImuFree;
          state.settled = false;
        }
        break;
      case EventType::BeginReturn:
        globalMode_ = GlobalMode::AutoReturn;
        for (RuntimeHand& state : states_) {
          state.animation = {};
          state.mode = HandMode::Returning;
          state.settled = false;
        }
        break;
      case EventType::ForceTimeTracking:
        globalMode_ = GlobalMode::TimeTracking;
        for (RuntimeHand& state : states_) {
          state.animation = {};
          state.mode = HandMode::TimeControlled;
        }
        break;
      case EventType::Recover:
        globalMode_ = GlobalMode::PhysicsPlay;
        snapshot_.lastError = ErrorCode::None;
        for (RuntimeHand& state : states_) {
          state.animation = {};
          state.flags |= SafetyRecovered;
          state.mode = HandMode::ImuFree;
        }
        break;
      case EventType::DriveImpulse:
      case EventType::ExternalImpulse: {
        RuntimeHand& state = states_[event.hand];
        const HandDefinition& hand = config_.hands[event.hand];
        if (hand.kind == HandKind::Rigid) {
          state.rigid.momentum += event.scalar;
#if DIAL_MUTATION_ID == 10
          state.rigid.momentum += event.scalar;
#endif
        } else {
          const Mass2 mass = flexibleMass(state.flexible.beta, hand.flex);
          const auto delta =
              inverseMultiply(mass, {event.scalar, 0.0F});
          state.flexible.qDot[0] += delta[0];
          state.flexible.qDot[1] += delta[1];
#if DIAL_MUTATION_ID == 10
          state.flexible.qDot[0] += delta[0];
          state.flexible.qDot[1] += delta[1];
#endif
        }
        (void)baseOmega;
        break;
      }
    }
  }
}

void PhysicalDialEngine::updateStateMachine(const StepInput& input,
                                            Vec3f baseOmega) noexcept {
  bool anyTouch = false;
  for (std::size_t i = 0; i < kHandCount; ++i) {
    const TouchTarget& touch = input.touchTargets[i];
    RuntimeHand& state = states_[i];
    const HandDefinition& hand = config_.hands[i];
    if (touch.active) {
      anyTouch = true;
      state.mode = HandMode::TouchConstrained;
      state.settled = false;
      globalMode_ = GlobalMode::PhysicsPlay;
    } else if (touch.release) {
      if (hand.kind == HandKind::Rigid) {
#if DIAL_MUTATION_ID == 7
        state.rigid.momentum = 0.0F;
#else
        state.rigid.momentum =
            hand.inertiaPivotKgM2 *
            (touch.angularVelocityRadSec + baseOmega.z);
#endif
      } else {
        state.flexible.qDot[0] = touch.angularVelocityRadSec;
      }
      state.mode = HandMode::ImuFree;
      state.flags |= SafetyTouchReleased;
      state.settled = false;
      globalMode_ = GlobalMode::PhysicsPlay;
    } else if (state.mode == HandMode::TouchConstrained) {
      state.mode = HandMode::ImuFree;
    }
  }

  if (input.intentionalInteraction || anyTouch) {
    idleTimeSec_ = 0.0F;
  } else {
    idleTimeSec_ += config_.baseDtSec;
  }
  if (globalMode_ == GlobalMode::PhysicsPlay && !anyTouch &&
      idleTimeSec_ >= config_.idleTimeoutSec) {
    globalMode_ = GlobalMode::AutoReturn;
    for (RuntimeHand& state : states_) {
      if (state.mode != HandMode::TouchConstrained) {
        state.mode = HandMode::Returning;
        state.settled = false;
      }
    }
  }
}

void PhysicalDialEngine::integrateRigid(
    std::size_t index, const BaseSubInterval& interval,
    const TimeTarget& time, const TouchTarget& touch, float driveTorque,
    float dt) noexcept {
  RuntimeHand& state = states_[index];
  const HandDefinition& hand = config_.hands[index];
  MomentumLedger momentum{};
  TorqueLedger torque{};
  const float oldMomentum = state.rigid.momentum;
  const float omegaStart =
      oldMomentum / hand.inertiaPivotKgM2 -
      interval.omegaBodyAtStartRadSec.z;
  const float thetaStart = state.rigid.angle.phase;
  const float thetaEval = thetaStart + 0.5F * omegaStart * dt;

  const Vec3f firstMoment =
      rotateZ(hand.centerOfMassAtZeroM * hand.massKg, thetaEval);
  const Vec3f firstMomentDerivative{-firstMoment.y, firstMoment.x, 0.0F};
  momentum.effectiveField =
      dot(firstMomentDerivative, interval.effectiveAccelImpulseMps);
#if DIAL_MUTATION_ID == 1
  momentum.effectiveField = -momentum.effectiveField;
#elif DIAL_MUTATION_ID == 2
  momentum.effectiveField *= 2.0F;
#endif
  const Vec3f axis = handAxis(thetaEval);
  const Vec3f tangent = handTangent(thetaEval);
  momentum.rotatingBase =
      -hand.inertiaPivotKgM2 *
      matrixQuadratic(interval.omegaOuterProductIntegral, axis, tangent);
#if DIAL_MUTATION_ID == 3
  momentum.rotatingBase +=
      -hand.inertiaPivotKgM2 *
      (interval.omegaBodyAtEndRadSec.z -
       interval.omegaBodyAtStartRadSec.z);
#endif

  torque.driveNm = driveTorque;
  torque.dampingNm = dampingTorque(hand, omegaStart);
  if ((state.mode == HandMode::Returning ||
       state.mode == HandMode::TimeControlled) &&
      time.valid && !touch.active) {
    const float error =
        shortestAngle(thetaEval, time.angleUnwrappedRad);
    torque.timeNm =
        limitedPd(error, time.angularVelocityRadSec - omegaStart,
                  hand.timeKpNmPerRad, hand.timeKdNmSecPerRad,
                  hand.maxTimeTorqueNm);
#if DIAL_MUTATION_ID == 6
    state.rigid.angle.set(time.angleUnwrappedRad);
#endif
  }
  if (touch.active) {
    const float error =
        shortestAngle(thetaEval, touch.angleUnwrappedRad);
    const float demand =
        hand.touchKpNmPerRad * error +
        hand.touchKdNmSecPerRad *
            (touch.angularVelocityRadSec - omegaStart);
    torque.touchNm =
        clampValue(demand, -hand.maxTouchTorqueNm, hand.maxTouchTorqueNm);
    if (std::abs(demand) > hand.maxTouchTorqueNm) {
      state.flags |= SafetyTouchReleased;
    }
  }
  torque.totalNm = torque.driveNm + torque.timeNm + torque.touchNm +
                   torque.dampingNm;
  momentum.drive = torque.driveNm * dt;
  momentum.time = torque.timeNm * dt;
  momentum.touch = torque.touchNm * dt;
  momentum.damping = torque.dampingNm * dt;
  const float unconstrained =
      oldMomentum + momentum.effectiveField + momentum.rotatingBase +
      momentum.drive + momentum.time + momentum.touch + momentum.damping;
  state.rigid.momentum = unconstrained;
  float omegaNew =
      state.rigid.momentum / hand.inertiaPivotKgM2 -
      interval.omegaBodyAtEndRadSec.z;
  const float energy =
      0.5F * hand.inertiaPivotKgM2 * omegaNew * omegaNew;
  float limitedOmega = clampValue(
      omegaNew, -hand.maxAngularSpeedRadSec, hand.maxAngularSpeedRadSec);
  if (limitedOmega != omegaNew) {
    state.flags |= SafetySpeedLimited;
  }
  const float energySpeed =
      std::sqrt(2.0F * hand.maxRelativeKineticEnergyJ /
                hand.inertiaPivotKgM2);
  const float energyLimited =
      clampValue(limitedOmega, -energySpeed, energySpeed);
  if (energy > hand.maxRelativeKineticEnergyJ) {
    state.flags |= SafetyEnergyLimited;
  }
  limitedOmega = energyLimited;
  state.rigid.momentum =
      hand.inertiaPivotKgM2 *
      (limitedOmega + interval.omegaBodyAtEndRadSec.z);
  momentum.safety = state.rigid.momentum - unconstrained;
  momentum.residual =
      (state.rigid.momentum - oldMomentum) -
      (momentum.effectiveField + momentum.rotatingBase + momentum.drive +
       momentum.time + momentum.touch + momentum.damping +
       momentum.safety);
#if DIAL_MUTATION_ID == 4
  state.rigid.angle.advance(omegaStart * dt);
#else
  state.rigid.angle.advance(limitedOmega * dt);
#endif

  torque.effectiveFieldNm = momentum.effectiveField / dt;
  torque.rotatingBaseNm = momentum.rotatingBase / dt;
  torque.totalNm += torque.effectiveFieldNm + torque.rotatingBaseNm;
  state.energy.kineticJ =
      0.5F * hand.inertiaPivotKgM2 * limitedOmega * limitedOmega;
  state.energy.gravityPotentialJ =
      -hand.massKg *
      dot(interval.gravityBodyAtEndMps2,
          rotateZ(hand.centerOfMassAtZeroM, state.rigid.angle.phase));
  state.energy.timePotentialJ =
      hand.timeKpNmPerRad *
      (1.0F -
       std::cos(shortestAngle(state.rigid.angle.phase,
                              time.angleUnwrappedRad)));
  state.energy.controlWorkJ += torque.timeNm * limitedOmega * dt;
  state.energy.touchWorkJ += torque.touchNm * limitedOmega * dt;
  state.energy.externalWorkJ += torque.driveNm * limitedOmega * dt;
  state.energy.dampingDissipationJ +=
      std::max(0.0F, -torque.dampingNm * limitedOmega * dt);
  snapshot_.hands[index].torque = torque;
  MomentumLedger& accumulated = snapshot_.hands[index].momentum;
  accumulated.effectiveField += momentum.effectiveField;
  accumulated.rotatingBase += momentum.rotatingBase;
  accumulated.drive += momentum.drive;
  accumulated.time += momentum.time;
  accumulated.touch += momentum.touch;
  accumulated.external += momentum.external;
  accumulated.elastic += momentum.elastic;
  accumulated.collision += momentum.collision;
  accumulated.damping += momentum.damping;
  accumulated.event += momentum.event;
  accumulated.safety += momentum.safety;
  accumulated.residual += momentum.residual;
}

void PhysicalDialEngine::integrateFlexible(
    std::size_t index, const BaseSubInterval& interval,
    const TimeTarget& time, const TouchTarget& touch, float driveTorque,
    float dt) noexcept {
  RuntimeHand& state = states_[index];
  const HandDefinition& hand = config_.hands[index];
  const FlexibleDefinition& f = hand.flex;
  const float theta = state.flexible.rootAngle.phase;
  const float beta = state.flexible.beta;
  const float thetaDot = state.flexible.qDot[0];
  const float betaDot = state.flexible.qDot[1];
  const float tip = theta + beta;
  const Vec3f uRoot = handAxis(theta);
  const Vec3f tRoot = handTangent(theta);
  const Vec3f uTip = handAxis(tip);
  const Vec3f tTip = handTangent(tip);
  const Vec3f r1 = uRoot * f.l1M;
  const Vec3f r2 = uRoot * f.d1M + uTip * f.l2M;
  const Vec3f j1Theta = tRoot * f.l1M;
  const Vec3f j2Theta = tRoot * f.d1M + tTip * f.l2M;
  const Vec3f j2Beta = tTip * f.l2M;
  const Vec3f acceleration =
      interval.effectiveAccelImpulseMps * (1.0F / dt);
  std::array<float, 2> generalized{
      f.m1Kg * dot(j1Theta, acceleration) +
          f.m2Kg * dot(j2Theta, acceleration),
      f.m2Kg * dot(j2Beta, acceleration)};

  const Vec3f omega =
      (interval.omegaBodyAtStartRadSec +
       interval.omegaBodyAtEndRadSec) *
      0.5F;
  const Vec3f alpha =
      (interval.omegaBodyAtEndRadSec -
       interval.omegaBodyAtStartRadSec) *
      (1.0F / dt);
  const Vec3f v1 = j1Theta * thetaDot;
  const Vec3f v2 = j2Theta * thetaDot + j2Beta * betaDot;
  const Vec3f pseudo1 =
      (cross(alpha, r1) + cross(omega, cross(omega, r1)) +
       cross(omega, v1) * 2.0F) *
      -1.0F;
  const Vec3f pseudo2 =
      (cross(alpha, r2) + cross(omega, cross(omega, r2)) +
       cross(omega, v2) * 2.0F) *
      -1.0F;
  const std::array<float, 2> rotating{
      f.m1Kg * dot(j1Theta, pseudo1) +
          f.m2Kg * dot(j2Theta, pseudo2),
      f.m2Kg * dot(j2Beta, pseudo2)};
  generalized[0] += rotating[0];
  generalized[1] += rotating[1];

  const float coupling = f.m2Kg * f.d1M * f.l2M;
  const float c1 =
      -coupling * std::sin(beta) *
      (2.0F * thetaDot * betaDot + betaDot * betaDot);
  const float c2 =
      coupling * std::sin(beta) * thetaDot * thetaDot;
  generalized[0] -= c1;
  generalized[1] -= c2;

  TorqueLedger ledger{};
  ledger.effectiveFieldNm =
      f.m1Kg * dot(j1Theta, acceleration) +
      f.m2Kg * dot(j2Theta, acceleration);
  ledger.rotatingBaseNm = rotating[0];
  ledger.driveNm = driveTorque;
  ledger.dampingNm = dampingTorque(hand, thetaDot);
  ledger.elasticNm = -f.bendStiffnessNmRad * beta;
  generalized[0] += driveTorque + ledger.dampingNm;
  generalized[1] += ledger.elasticNm - f.bendDampingNmSecRad * betaDot;
#if DIAL_MUTATION_ID == 8
  generalized[0] -= ledger.elasticNm;
#endif

  if ((state.mode == HandMode::Returning ||
       state.mode == HandMode::TimeControlled) &&
      time.valid && !touch.active) {
    const float error = shortestAngle(theta, time.angleUnwrappedRad);
    ledger.timeNm =
        limitedPd(error, time.angularVelocityRadSec - thetaDot,
                  hand.timeKpNmPerRad, hand.timeKdNmSecPerRad,
                  hand.maxTimeTorqueNm);
    generalized[0] += ledger.timeNm;
  }
  if (touch.active) {
    const float error = shortestAngle(tip, touch.angleUnwrappedRad);
    const float tipVelocity = thetaDot + betaDot;
    const float demand =
        hand.touchKpNmPerRad * error +
        hand.touchKdNmSecPerRad *
            (touch.angularVelocityRadSec - tipVelocity);
    ledger.touchNm =
        clampValue(demand, -hand.maxTouchTorqueNm, hand.maxTouchTorqueNm);
    generalized[0] += ledger.touchNm;
    generalized[1] += ledger.touchNm;
    const float bendEnergy =
        0.5F * f.bendStiffnessNmRad * beta * beta;
    if (std::abs(beta) > f.releaseAngleRad ||
        bendEnergy > f.releaseEnergyJ ||
        std::abs(demand) > hand.maxTouchTorqueNm) {
      state.mode = HandMode::ImuFree;
      state.flags |= SafetyTouchReleased;
    }
  }

  const Mass2 mass = flexibleMass(beta, f);
  const auto accelerationQ = inverseMultiply(mass, generalized);
  state.flexible.qDot[0] += accelerationQ[0] * dt;
  state.flexible.qDot[1] += accelerationQ[1] * dt;
  const float rootLimited =
      clampValue(state.flexible.qDot[0], -hand.maxAngularSpeedRadSec,
                 hand.maxAngularSpeedRadSec);
  if (rootLimited != state.flexible.qDot[0]) {
    state.flags |= SafetySpeedLimited;
    state.flexible.qDot[0] = rootLimited;
  }
  state.flexible.rootAngle.advance(state.flexible.qDot[0] * dt);
  state.flexible.beta += state.flexible.qDot[1] * dt;
  if (!std::isfinite(state.flexible.beta) ||
      !std::isfinite(state.flexible.qDot[0]) ||
      !std::isfinite(state.flexible.qDot[1])) {
    state.flexible = {};
    state.flags |= SafetyRecovered;
    globalMode_ = GlobalMode::Degraded;
    snapshot_.lastError = ErrorCode::NonFiniteInput;
  }

  ledger.totalNm = ledger.effectiveFieldNm + ledger.rotatingBaseNm +
                   ledger.driveNm + ledger.timeNm + ledger.touchNm +
                   ledger.elasticNm + ledger.dampingNm;
  snapshot_.hands[index].torque = ledger;
  MomentumLedger momentum{};
  momentum.effectiveField = ledger.effectiveFieldNm * dt;
  momentum.rotatingBase = ledger.rotatingBaseNm * dt;
  momentum.drive = ledger.driveNm * dt;
  momentum.time = ledger.timeNm * dt;
  momentum.touch = ledger.touchNm * dt;
  momentum.elastic = ledger.elasticNm * dt;
  momentum.damping = ledger.dampingNm * dt;
  MomentumLedger& accumulated = snapshot_.hands[index].momentum;
  accumulated.effectiveField += momentum.effectiveField;
  accumulated.rotatingBase += momentum.rotatingBase;
  accumulated.drive += momentum.drive;
  accumulated.time += momentum.time;
  accumulated.touch += momentum.touch;
  accumulated.external += momentum.external;
  accumulated.elastic += momentum.elastic;
  accumulated.collision += momentum.collision;
  accumulated.damping += momentum.damping;
  accumulated.event += momentum.event;
  accumulated.safety += momentum.safety;
  accumulated.residual += momentum.residual;

  const float kinetic =
      0.5F *
      (mass.m11 * state.flexible.qDot[0] * state.flexible.qDot[0] +
       2.0F * mass.m12 * state.flexible.qDot[0] *
           state.flexible.qDot[1] +
       mass.m22 * state.flexible.qDot[1] * state.flexible.qDot[1]);
  state.energy.kineticJ = kinetic;
  state.energy.flexPotentialJ =
      0.5F * f.bendStiffnessNmRad * state.flexible.beta *
      state.flexible.beta;
  state.energy.gravityPotentialJ =
      -dot(interval.gravityBodyAtEndMps2,
           r1 * f.m1Kg + r2 * f.m2Kg);
  state.energy.controlWorkJ += ledger.timeNm * state.flexible.qDot[0] * dt;
  state.energy.touchWorkJ +=
      ledger.touchNm *
      (state.flexible.qDot[0] + state.flexible.qDot[1]) * dt;
  state.energy.dampingDissipationJ +=
      std::max(0.0F,
               -(ledger.dampingNm * state.flexible.qDot[0] -
                 f.bendDampingNmSecRad * state.flexible.qDot[1] *
                     state.flexible.qDot[1]) *
                   dt);
}

void PhysicalDialEngine::applyAnimation(std::size_t index,
                                        const TimeTarget& target,
                                        Vec3f omegaEnd,
                                        std::uint64_t nowUs) noexcept {
  RuntimeHand& state = states_[index];
  const HandDefinition& hand = config_.hands[index];
  AnimationState& animation = state.animation;
  double currentAngle =
      hand.kind == HandKind::Rigid ? state.rigid.angle.value()
                                   : state.flexible.rootAngle.value();
  float currentVelocity =
      hand.kind == HandKind::Rigid
          ? state.rigid.momentum / hand.inertiaPivotKgM2 - omegaEnd.z
          : state.flexible.qDot[0];
  if (!animation.active) {
    animation.active = true;
    animation.startAngle = currentAngle;
    animation.startVelocity =
        config_.animationPreserveEntryVelocity ? currentVelocity : 0.0F;
    animation.startUs = nowUs;
    animation.durationUs = static_cast<std::uint64_t>(
        config_.animationDurationSec * 1.0e6F);
    state.settled = false;
  }
  const float duration =
      std::max(static_cast<float>(animation.durationUs) * 1.0e-6F,
               1.0e-4F);
  const float elapsed =
      static_cast<float>(nowUs - animation.startUs) * 1.0e-6F;
  const float s = clampValue(elapsed / duration, 0.0F, 1.0F);
  const double targetAtEnd =
      static_cast<double>(target.angleUnwrappedRad) +
      static_cast<double>(target.angularVelocityRadSec) *
          static_cast<double>(std::max(0.0F, duration - elapsed));
  const double destination =
      animation.startAngle +
      static_cast<double>(shortestAngle(
          static_cast<float>(animation.startAngle),
          static_cast<float>(targetAtEnd)));
  const double m0 =
      static_cast<double>(animation.startVelocity * duration);
  const double m1 =
      static_cast<double>(target.angularVelocityRadSec * duration);
  const double d = destination - animation.startAngle - m0;
  const double v = m1 - m0;
  const double a3 = 10.0 * d - 4.0 * v;
  const double a4 = 7.0 * v - 15.0 * d;
  const double a5 = 6.0 * d - 3.0 * v;
  const double sd = static_cast<double>(s);
  const double position =
      animation.startAngle + m0 * sd + a3 * sd * sd * sd +
      a4 * sd * sd * sd * sd + a5 * sd * sd * sd * sd * sd;
  const double derivative =
      m0 + 3.0 * a3 * sd * sd + 4.0 * a4 * sd * sd * sd +
      5.0 * a5 * sd * sd * sd * sd;
  const float velocity = static_cast<float>(derivative / duration);
  if (hand.kind == HandKind::Rigid) {
    state.rigid.angle.set(position);
    state.rigid.momentum =
        hand.inertiaPivotKgM2 * (velocity + omegaEnd.z);
  } else {
    state.flexible.rootAngle.set(position);
    state.flexible.qDot[0] = velocity;
  }
  if (s >= 1.0F) {
    animation.active = false;
    state.settled = true;
    state.mode = HandMode::TimeControlled;
  }
}

void PhysicalDialEngine::solveContacts(const StepInput& input,
                                       Vec3f omegaEnd) noexcept {
  std::array<std::uint64_t, kMaxContacts> consumedIds{};
  std::size_t consumedCount = 0;
  const std::size_t count = std::min(input.contactCount, kMaxContacts);
  for (std::size_t contactIndex = 0; contactIndex < count; ++contactIndex) {
    const ContactConstraint& contact = input.contacts[contactIndex];
    if (!contact.active || !validHandIndex(contact.handA) ||
        (contact.handB >= 0 && !validHandIndex(contact.handB)) ||
        !std::isfinite(contact.gapM) ||
        !std::isfinite(contact.restitution) ||
        contact.restitution < 0.0F || contact.restitution > 1.0F) {
      snapshot_.lastError = ErrorCode::ContactRejected;
      continue;
    }
    bool duplicate = false;
    for (std::size_t id = 0; id < consumedCount; ++id) {
      duplicate = duplicate || consumedIds[id] == contact.eventId;
    }
    if (duplicate) {
      snapshot_.lastError = ErrorCode::ContactRejected;
      continue;
    }
    consumedIds[consumedCount++] = contact.eventId;

    auto velocityAndInverseMass =
        [&](std::int8_t handIndex,
            const std::array<float, 2>& jacobian) {
          std::array<float, 2> result{};
          if (!validHandIndex(handIndex)) {
            return result;
          }
          const std::size_t i = static_cast<std::size_t>(handIndex);
          const HandDefinition& hand = config_.hands[i];
          RuntimeHand& state = states_[i];
          if (hand.kind == HandKind::Rigid) {
            const float velocity =
                state.rigid.momentum / hand.inertiaPivotKgM2 - omegaEnd.z;
            result[0] = jacobian[0] * velocity;
            result[1] =
                jacobian[0] * jacobian[0] / hand.inertiaPivotKgM2;
          } else {
            const Mass2 mass =
                flexibleMass(state.flexible.beta, hand.flex);
            const auto inverseJ = inverseMultiply(mass, jacobian);
            result[0] =
                jacobian[0] * state.flexible.qDot[0] +
                jacobian[1] * state.flexible.qDot[1];
            result[1] =
                jacobian[0] * inverseJ[0] +
                jacobian[1] * inverseJ[1];
          }
          return result;
        };

    const auto a =
        velocityAndInverseMass(contact.handA, contact.jacobianA);
    const auto b =
        velocityAndInverseMass(contact.handB, contact.jacobianB);
    const float relativeVelocity = a[0] + b[0];
    const float inverseMass = a[1] + b[1];
    if (!(inverseMass > kMinPositive)) {
      snapshot_.lastError = ErrorCode::ContactRejected;
      continue;
    }
    const float separationBias =
        contact.gapM < 0.0F
            ? std::max(contact.biasVelocityRadSec,
                       -0.2F * contact.gapM / config_.baseDtSec)
            : 0.0F;
    const float desiredVelocity =
        std::max(-contact.restitution * relativeVelocity, separationBias);
    const float lambda =
        std::max(0.0F, (desiredVelocity - relativeVelocity) / inverseMass);
    if (lambda == 0.0F) {
      continue;
    }

    auto applyImpulse = [&](std::int8_t handIndex,
                            const std::array<float, 2>& jacobian) {
      if (!validHandIndex(handIndex)) {
        return;
      }
      const std::size_t i = static_cast<std::size_t>(handIndex);
      const HandDefinition& hand = config_.hands[i];
      RuntimeHand& state = states_[i];
      float energyBefore = 0.0F;
      if (hand.kind == HandKind::Rigid) {
        const float velocity =
            state.rigid.momentum / hand.inertiaPivotKgM2 - omegaEnd.z;
        energyBefore =
            0.5F * hand.inertiaPivotKgM2 * velocity * velocity;
      } else {
        const Mass2 mass =
            flexibleMass(state.flexible.beta, hand.flex);
        energyBefore =
            0.5F *
            (mass.m11 * state.flexible.qDot[0] * state.flexible.qDot[0] +
             2.0F * mass.m12 * state.flexible.qDot[0] *
                 state.flexible.qDot[1] +
             mass.m22 * state.flexible.qDot[1] *
                 state.flexible.qDot[1]);
      }
      if (hand.kind == HandKind::Rigid) {
        state.rigid.momentum += jacobian[0] * lambda;
      } else {
        const Mass2 mass =
            flexibleMass(state.flexible.beta, hand.flex);
        const auto delta =
            inverseMultiply(mass,
                            {jacobian[0] * lambda, jacobian[1] * lambda});
        state.flexible.qDot[0] += delta[0];
        state.flexible.qDot[1] += delta[1];
      }
      state.flags |= SafetyContactApplied;
      snapshot_.hands[i].momentum.collision += jacobian[0] * lambda;
      float energyAfter = 0.0F;
      if (hand.kind == HandKind::Rigid) {
        const float velocity =
            state.rigid.momentum / hand.inertiaPivotKgM2 - omegaEnd.z;
        energyAfter =
            0.5F * hand.inertiaPivotKgM2 * velocity * velocity;
      } else {
        const Mass2 mass =
            flexibleMass(state.flexible.beta, hand.flex);
        energyAfter =
            0.5F *
            (mass.m11 * state.flexible.qDot[0] * state.flexible.qDot[0] +
             2.0F * mass.m12 * state.flexible.qDot[0] *
                 state.flexible.qDot[1] +
             mass.m22 * state.flexible.qDot[1] *
                 state.flexible.qDot[1]);
      }
      state.energy.collisionEnergyDeltaJ += energyAfter - energyBefore;
    };
    applyImpulse(contact.handA, contact.jacobianA);
    applyImpulse(contact.handB, contact.jacobianB);
#if DIAL_MUTATION_ID == 9
    applyImpulse(contact.handA, contact.jacobianA);
    applyImpulse(contact.handB, contact.jacobianB);
#endif
  }
}

bool PhysicalDialEngine::step(const BaseInterval& base,
                              const StepInput& input) noexcept {
  if (!configured_ || input.eventCount > kMaxEvents ||
      input.contactCount > kMaxContacts || !validateInterval(base)) {
    snapshot_.lastError =
        configured_ ? ErrorCode::InvalidInterval
                    : ErrorCode::InvalidConfiguration;
    ++snapshot_.rejectedIntervals;
    return false;
  }
  if ((base.qualityFlags &
       (QualityDuplicate | QualityOutOfOrder | QualitySaturated)) != 0U) {
    snapshot_.lastError = ErrorCode::InvalidInterval;
    ++snapshot_.rejectedIntervals;
    return false;
  }
  if ((base.qualityFlags & QualityGap) != 0U) {
    globalMode_ = GlobalMode::Degraded;
    ++snapshot_.degradationCount;
  }

  snapshot_.lastError = ErrorCode::None;
  for (HandSnapshot& hand : snapshot_.hands) {
    hand.momentum = {};
    hand.torque = {};
  }
  applyEvents(input, base.quarter[0].omegaBodyAtStartRadSec);
  updateStateMachine(input, base.quarter[0].omegaBodyAtStartRadSec);

  const std::size_t substeps = config_.substeps;
  const std::size_t quartersPerSubstep = kBaseQuarterCount / substeps;
  for (std::size_t substep = 0; substep < substeps; ++substep) {
    const BaseSubInterval interval =
        coalesce(base, substep * quartersPerSubstep,
                 quartersPerSubstep);
    const float dt = intervalSeconds(interval);
    for (std::size_t i = 0; i < kHandCount; ++i) {
      RuntimeHand& state = states_[i];
      const bool wantsAnimation =
          globalMode_ == GlobalMode::AutoReturn &&
          !state.settled &&
          (config_.returnStrategy == ReturnStrategy::Animation ||
           (config_.returnStrategy == ReturnStrategy::Hybrid &&
            std::abs(shortestAngle(
                static_cast<float>(
                    config_.hands[i].kind == HandKind::Rigid
                        ? state.rigid.angle.value()
                        : state.flexible.rootAngle.value()),
                input.timeTargets[i].angleUnwrappedRad)) <
                config_.hybridCaptureAngleRad));
      if (wantsAnimation) {
        applyAnimation(i, input.timeTargets[i],
                       interval.omegaBodyAtEndRadSec, interval.endUs);
      } else if (config_.hands[i].kind == HandKind::Rigid) {
        integrateRigid(i, interval, input.timeTargets[i],
                       input.touchTargets[i], input.driveTorqueNm[i], dt);
      } else {
        integrateFlexible(i, interval, input.timeTargets[i],
                          input.touchTargets[i], input.driveTorqueNm[i],
                          dt);
      }

      const float angle =
          static_cast<float>(config_.hands[i].kind == HandKind::Rigid
                                 ? state.rigid.angle.value()
                                 : state.flexible.rootAngle.value());
      const float velocity =
          config_.hands[i].kind == HandKind::Rigid
              ? state.rigid.momentum /
                        config_.hands[i].inertiaPivotKgM2 -
                    interval.omegaBodyAtEndRadSec.z
              : state.flexible.qDot[0];
      const float angleError =
          std::abs(shortestAngle(angle,
                                 input.timeTargets[i].angleUnwrappedRad));
      const float velocityError =
          std::abs(velocity -
                   input.timeTargets[i].angularVelocityRadSec);
      if (angleError < config_.settleAngleRad &&
          velocityError < config_.settleVelocityRadSec) {
        state.settledTimeSec += dt;
        if (state.settledTimeSec >= config_.settleDurationSec) {
          state.settled = true;
        }
      } else {
        state.settledTimeSec = 0.0F;
        state.settled = false;
      }
    }
  }
  solveContacts(input, base.quarter[3].omegaBodyAtEndRadSec);

  if (globalMode_ == GlobalMode::AutoReturn) {
    bool allSettled = true;
    for (const RuntimeHand& state : states_) {
      allSettled = allSettled && state.settled;
    }
    if (allSettled) {
      globalMode_ = GlobalMode::TimeTracking;
      for (RuntimeHand& state : states_) {
        state.mode = HandMode::TimeControlled;
      }
    }
  }

  expectedStartUs_ = base.endUs;
  ++snapshot_.consumedIntervals;
  publish(base);
  return true;
}

void PhysicalDialEngine::publish(const BaseInterval& base) noexcept {
  ++snapshot_.sequence;
  snapshot_.timestampUs = base.endUs;
  snapshot_.globalMode = globalMode_;
  for (std::size_t i = 0; i < kHandCount; ++i) {
    const HandDefinition& hand = config_.hands[i];
    const RuntimeHand& state = states_[i];
    HandSnapshot& output = snapshot_.hands[i];
    const Vec3f omega =
        base.endUs == 0U ? Vec3f{}
                         : base.quarter[3].omegaBodyAtEndRadSec;
    output.kind = hand.kind;
    output.mode = state.mode;
    output.stateValid = true;
    output.settled = state.settled;
    output.safetyFlags = state.flags;
    if (hand.kind == HandKind::Rigid) {
      output.angleUnwrappedRad = state.rigid.angle.value();
      output.reducedMomentumKgM2RadSec = state.rigid.momentum;
      output.angularVelocityRadSec =
          state.rigid.momentum / hand.inertiaPivotKgM2 - omega.z;
      output.momentumAuthoritative = !state.animation.active;
      output.bendAngleRad = 0.0F;
      output.bendVelocityRadSec = 0.0F;
    } else {
      output.angleUnwrappedRad = state.flexible.rootAngle.value();
      output.angularVelocityRadSec = state.flexible.qDot[0];
      output.reducedMomentumKgM2RadSec = 0.0F;
      output.momentumAuthoritative = false;
      output.bendAngleRad = state.flexible.beta;
      output.bendVelocityRadSec = state.flexible.qDot[1];
    }
    const float angle = static_cast<float>(output.angleUnwrappedRad);
    output.axisBody = handAxis(angle);
    output.tangentBody = handTangent(angle);
    output.centerOfMassBodyM =
        rotateZ(hand.centerOfMassAtZeroM, angle);
    output.tipBodyM = output.axisBody * hand.visualLengthM;
    output.energy = state.energy;
  }
}

const EngineSnapshot& PhysicalDialEngine::snapshot() const noexcept {
  return snapshot_;
}

const EngineConfig& PhysicalDialEngine::config() const noexcept {
  return config_;
}

std::size_t PhysicalDialEngine::coreStateBytes() const noexcept {
  return sizeof(*this);
}

std::uint64_t deterministicHash(const EngineSnapshot& snapshot) noexcept {
  std::uint64_t hash = 1469598103934665603ULL;
  auto addByte = [&](std::uint8_t byte) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  };
  auto addU64 = [&](std::uint64_t value) {
    for (unsigned int shift = 0; shift < 64U; shift += 8U) {
      addByte(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
  };
  auto addU32 = [&](std::uint32_t value) {
    addU64(value);
  };
  auto addFloat = [&](float value) {
    addU32(std::bit_cast<std::uint32_t>(value));
  };
  auto addDouble = [&](double value) {
    addU64(std::bit_cast<std::uint64_t>(value));
  };
  auto addVec = [&](Vec3f value) {
    addFloat(value.x);
    addFloat(value.y);
    addFloat(value.z);
  };
  auto addTorque = [&](const TorqueLedger& value) {
    addFloat(value.effectiveFieldNm);
    addFloat(value.rotatingBaseNm);
    addFloat(value.driveNm);
    addFloat(value.timeNm);
    addFloat(value.touchNm);
    addFloat(value.externalNm);
    addFloat(value.elasticNm);
    addFloat(value.collisionNm);
    addFloat(value.dampingNm);
    addFloat(value.totalNm);
  };
  auto addMomentum = [&](const MomentumLedger& value) {
    addFloat(value.effectiveField);
    addFloat(value.rotatingBase);
    addFloat(value.drive);
    addFloat(value.time);
    addFloat(value.touch);
    addFloat(value.external);
    addFloat(value.elastic);
    addFloat(value.collision);
    addFloat(value.damping);
    addFloat(value.event);
    addFloat(value.safety);
    addFloat(value.residual);
  };
  auto addEnergy = [&](const EnergyLedger& value) {
    addFloat(value.kineticJ);
    addFloat(value.gravityPotentialJ);
    addFloat(value.timePotentialJ);
    addFloat(value.flexPotentialJ);
    addFloat(value.externalWorkJ);
    addFloat(value.controlWorkJ);
    addFloat(value.touchWorkJ);
    addFloat(value.dampingDissipationJ);
    addFloat(value.collisionEnergyDeltaJ);
  };

  addU64(snapshot.sequence);
  addU64(snapshot.timestampUs);
  addByte(static_cast<std::uint8_t>(snapshot.globalMode));
  addU32(static_cast<std::uint32_t>(snapshot.lastError));
  addU32(snapshot.rejectedIntervals);
  addU32(snapshot.consumedIntervals);
  addU32(snapshot.degradationCount);
  for (const HandSnapshot& hand : snapshot.hands) {
    addByte(static_cast<std::uint8_t>(hand.kind));
    addByte(static_cast<std::uint8_t>(hand.mode));
    addByte(hand.stateValid ? 1U : 0U);
    addByte(hand.momentumAuthoritative ? 1U : 0U);
    addByte(hand.settled ? 1U : 0U);
    addDouble(hand.angleUnwrappedRad);
    addFloat(hand.angularVelocityRadSec);
    addFloat(hand.reducedMomentumKgM2RadSec);
    addFloat(hand.bendAngleRad);
    addFloat(hand.bendVelocityRadSec);
    addVec(hand.axisBody);
    addVec(hand.tangentBody);
    addVec(hand.centerOfMassBodyM);
    addVec(hand.tipBodyM);
    addTorque(hand.torque);
    addMomentum(hand.momentum);
    addEnergy(hand.energy);
    addU32(hand.safetyFlags);
  }
  return hash;
}

}  // namespace dial
