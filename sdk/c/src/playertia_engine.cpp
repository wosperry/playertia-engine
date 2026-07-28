#include "playertia/engine.h"

#include "dial/physics.hpp"

#include <new>

struct playertia_engine {
  dial::PhysicalDialEngine implementation{};
};

namespace {

dial::Vec3f convert(playertia_vec3 value) noexcept {
  return {value.x, value.y, value.z};
}

playertia_vec3 convert(dial::Vec3f value) noexcept {
  return {value.x, value.y, value.z};
}

dial::BaseInterval convert(const playertia_base_interval& source) noexcept {
  dial::BaseInterval target{};
  target.startUs = source.start_us;
  target.endUs = source.end_us;
  target.qualityFlags = source.quality_flags;
  for (std::size_t q = 0; q < dial::kBaseQuarterCount; ++q) {
    const auto& from = source.quarter[q];
    auto& to = target.quarter[q];
    to.startUs = from.start_us;
    to.endUs = from.end_us;
    to.effectiveAccelImpulseMps =
        convert(from.effective_accel_impulse_mps);
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        to.omegaOuterProductIntegral.v[row][column] =
            from.omega_outer_product_integral[row * 3 + column];
      }
    }
    to.omegaBodyAtStartRadSec =
        convert(from.omega_body_at_start_rad_sec);
    to.omegaBodyAtEndRadSec = convert(from.omega_body_at_end_rad_sec);
    to.gravityBodyAtEndMps2 = convert(from.gravity_body_at_end_mps2);
    to.qualityFlags = from.quality_flags;
  }
  return target;
}

bool valid_event_type(std::uint8_t type) noexcept {
  return type <= static_cast<std::uint8_t>(PLAYERTIA_EVENT_EXTERNAL_IMPULSE);
}

dial::StepInput convert(const playertia_step_input& source) noexcept {
  dial::StepInput target{};
  for (std::size_t hand = 0; hand < dial::kHandCount; ++hand) {
    const auto& time = source.time_targets[hand];
    target.timeTargets[hand] = {
        time.angle_unwrapped_rad,
        time.angular_velocity_rad_sec,
        time.angular_acceleration_rad_sec2,
        time.timestamp_us,
        time.valid != 0};

    const auto& touch = source.touch_targets[hand];
    target.touchTargets[hand] = {
        touch.active != 0,
        touch.release != 0,
        touch.angle_unwrapped_rad,
        touch.angular_velocity_rad_sec,
        touch.confidence,
        touch.timestamp_us};
    target.driveTorqueNm[hand] = source.drive_torque_nm[hand];
  }

  target.eventCount = source.event_count;
  for (std::size_t index = 0; index < source.event_count; ++index) {
    const auto& from = source.events[index];
    target.events[index] = {
        static_cast<dial::EventType>(from.type),
        from.hand,
        from.scalar,
        from.timestamp_us};
  }

  target.contactCount = source.contact_count;
  for (std::size_t index = 0; index < source.contact_count; ++index) {
    const auto& from = source.contacts[index];
    target.contacts[index] = {
        from.hand_a,
        from.hand_b,
        {from.jacobian_a[0], from.jacobian_a[1]},
        {from.jacobian_b[0], from.jacobian_b[1]},
        from.gap_m,
        from.restitution,
        from.bias_velocity_rad_sec,
        from.event_id,
        from.active != 0};
  }
  target.intentionalInteraction = source.intentional_interaction != 0;
  return target;
}

}  // namespace

extern "C" {

uint32_t playertia_engine_get_abi_version(void) {
  return PLAYERTIA_ABI_VERSION;
}

size_t playertia_engine_get_state_size(void) {
  return sizeof(playertia_engine);
}

playertia_result playertia_engine_create(uint32_t abi_version, playertia_engine** out_engine) {
  if (out_engine == nullptr) {
    return PLAYERTIA_INVALID_ARGUMENT;
  }
  *out_engine = nullptr;
  if (abi_version != PLAYERTIA_ABI_VERSION) {
    return PLAYERTIA_ABI_MISMATCH;
  }
  auto* engine = new (std::nothrow) playertia_engine{};
  if (engine == nullptr) {
    return PLAYERTIA_ALLOCATION_FAILED;
  }
  if (!engine->implementation.configure(dial::makeDefaultConfig())) {
    delete engine;
    return PLAYERTIA_ENGINE_REJECTED_INPUT;
  }
  *out_engine = engine;
  return PLAYERTIA_OK;
}

void playertia_engine_destroy(playertia_engine* engine) {
  delete engine;
}

playertia_result playertia_engine_reset(
    playertia_engine* engine,
    const double angles_rad[PLAYERTIA_HAND_COUNT],
    const float relative_velocities_rad_sec[PLAYERTIA_HAND_COUNT],
    playertia_vec3 base_omega_rad_sec) {
  if (engine == nullptr || angles_rad == nullptr ||
      relative_velocities_rad_sec == nullptr) {
    return PLAYERTIA_INVALID_ARGUMENT;
  }
  std::array<double, dial::kHandCount> angles{};
  std::array<float, dial::kHandCount> velocities{};
  for (std::size_t hand = 0; hand < dial::kHandCount; ++hand) {
    angles[hand] = angles_rad[hand];
    velocities[hand] = relative_velocities_rad_sec[hand];
  }
  engine->implementation.reset(
      angles, velocities, convert(base_omega_rad_sec));
  return PLAYERTIA_OK;
}

playertia_result playertia_engine_step(
    playertia_engine* engine,
    const playertia_base_interval* interval,
    const playertia_step_input* input) {
  if (engine == nullptr || interval == nullptr || input == nullptr) {
    return PLAYERTIA_INVALID_ARGUMENT;
  }
  if (interval->abi_version != PLAYERTIA_ABI_VERSION ||
      input->abi_version != PLAYERTIA_ABI_VERSION) {
    return PLAYERTIA_ABI_MISMATCH;
  }
  if (interval->struct_size !=
          static_cast<std::uint32_t>(sizeof(playertia_base_interval)) ||
      input->struct_size !=
          static_cast<std::uint32_t>(sizeof(playertia_step_input)) ||
      input->event_count > PLAYERTIA_MAX_EVENTS ||
      input->contact_count > PLAYERTIA_MAX_CONTACTS) {
    return PLAYERTIA_INVALID_ARGUMENT;
  }
  for (std::size_t index = 0; index < input->event_count; ++index) {
    if (!valid_event_type(input->events[index].type)) {
      return PLAYERTIA_INVALID_ARGUMENT;
    }
  }
  const auto native_interval = convert(*interval);
  const auto native_input = convert(*input);
  return engine->implementation.step(native_interval, native_input)
             ? PLAYERTIA_OK
             : PLAYERTIA_ENGINE_REJECTED_INPUT;
}

playertia_result playertia_engine_get_snapshot(
    const playertia_engine* engine,
    playertia_snapshot* out_snapshot,
    uint32_t out_snapshot_size) {
  if (engine == nullptr || out_snapshot == nullptr ||
      out_snapshot_size !=
          static_cast<std::uint32_t>(sizeof(playertia_snapshot))) {
    return PLAYERTIA_INVALID_ARGUMENT;
  }
  const auto& source = engine->implementation.snapshot();
  *out_snapshot = {};
  out_snapshot->abi_version = PLAYERTIA_ABI_VERSION;
  out_snapshot->struct_size =
      static_cast<std::uint32_t>(sizeof(playertia_snapshot));
  out_snapshot->sequence = source.sequence;
  out_snapshot->timestamp_us = source.timestampUs;
  out_snapshot->deterministic_hash = dial::deterministicHash(source);
  out_snapshot->rejected_intervals = source.rejectedIntervals;
  out_snapshot->consumed_intervals = source.consumedIntervals;
  out_snapshot->degradation_count = source.degradationCount;
  out_snapshot->last_error = static_cast<std::uint16_t>(source.lastError);
  out_snapshot->global_mode =
      static_cast<std::uint8_t>(source.globalMode);
  for (std::size_t hand = 0; hand < dial::kHandCount; ++hand) {
    const auto& from = source.hands[hand];
    auto& to = out_snapshot->hands[hand];
    to.angle_unwrapped_rad = from.angleUnwrappedRad;
    to.angular_velocity_rad_sec = from.angularVelocityRadSec;
    to.reduced_momentum_kg_m2_rad_sec =
        from.reducedMomentumKgM2RadSec;
    to.bend_angle_rad = from.bendAngleRad;
    to.bend_velocity_rad_sec = from.bendVelocityRadSec;
    to.axis_body = convert(from.axisBody);
    to.center_of_mass_body_m = convert(from.centerOfMassBodyM);
    to.tip_body_m = convert(from.tipBodyM);
    to.safety_flags = from.safetyFlags;
    to.kind = static_cast<std::uint8_t>(from.kind);
    to.mode = static_cast<std::uint8_t>(from.mode);
    to.state_valid = static_cast<std::uint8_t>(from.stateValid);
    to.momentum_authoritative =
        static_cast<std::uint8_t>(from.momentumAuthoritative);
    to.settled = static_cast<std::uint8_t>(from.settled);
  }
  return PLAYERTIA_OK;
}

}  // extern "C"
