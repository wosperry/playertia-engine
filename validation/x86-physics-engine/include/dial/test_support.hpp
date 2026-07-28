#pragma once

#include "dial/physics.hpp"

#include <cmath>
#include <cstdint>

namespace dial::test {

inline BaseInterval makeBase(std::uint64_t startUs, Vec3f effectiveAcceleration,
                             Vec3f omegaStart = {}, Vec3f omegaEnd = {},
                             Vec3f gravity = {0.0F, 0.0F, -9.80665F},
                             std::uint32_t quality = 0U) noexcept {
  BaseInterval base{};
  base.startUs = startUs;
  base.endUs = startUs + 10'000U;
  base.qualityFlags = quality;
  for (std::size_t quarter = 0; quarter < kBaseQuarterCount; ++quarter) {
    BaseSubInterval& part = base.quarter[quarter];
    part.startUs = startUs + quarter * 2'500U;
    part.endUs = part.startUs + 2'500U;
    const float a0 = static_cast<float>(quarter) / 4.0F;
    const float a1 = static_cast<float>(quarter + 1U) / 4.0F;
    part.omegaBodyAtStartRadSec =
        omegaStart * (1.0F - a0) + omegaEnd * a0;
    part.omegaBodyAtEndRadSec =
        omegaStart * (1.0F - a1) + omegaEnd * a1;
    const Vec3f midpoint =
        (part.omegaBodyAtStartRadSec + part.omegaBodyAtEndRadSec) *
        0.5F;
    part.effectiveAccelImpulseMps = effectiveAcceleration * 0.0025F;
    const float components[3]{midpoint.x, midpoint.y, midpoint.z};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        part.omegaOuterProductIntegral.v[row][column] =
            components[row] * components[column] * 0.0025F;
      }
    }
    part.gravityBodyAtEndMps2 = gravity;
    part.qualityFlags = quality;
  }
  return base;
}

inline StepInput makeInput(float targetAngle = 0.0F,
                           float targetVelocity = 0.0F) noexcept {
  StepInput input{};
  for (TimeTarget& target : input.timeTargets) {
    target.angleUnwrappedRad = targetAngle;
    target.angularVelocityRadSec = targetVelocity;
    target.valid = true;
  }
  return input;
}

inline bool finiteSnapshot(const EngineSnapshot& snapshot) noexcept {
  for (const HandSnapshot& hand : snapshot.hands) {
    if (!std::isfinite(hand.angleUnwrappedRad) ||
        !std::isfinite(hand.angularVelocityRadSec) ||
        !std::isfinite(hand.reducedMomentumKgM2RadSec) ||
        !std::isfinite(hand.bendAngleRad) ||
        !std::isfinite(hand.bendVelocityRadSec) ||
        !finite(hand.axisBody) || !finite(hand.tipBodyM)) {
      return false;
    }
  }
  return true;
}

class XorShift64 {
 public:
  explicit XorShift64(std::uint64_t seed) noexcept
      : state_(seed == 0U ? 0x9e3779b97f4a7c15ULL : seed) {}

  std::uint64_t next() noexcept {
    std::uint64_t value = state_;
    value ^= value << 13U;
    value ^= value >> 7U;
    value ^= value << 17U;
    state_ = value;
    return value;
  }

  float uniform(float low, float high) noexcept {
    const double unit =
        static_cast<double>(next() >> 11U) *
        (1.0 / static_cast<double>(1ULL << 53U));
    return low + static_cast<float>(unit) * (high - low);
  }

 private:
  std::uint64_t state_;
};

}  // namespace dial::test

