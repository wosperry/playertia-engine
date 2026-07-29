#pragma once

#include <array>
#include <cstdint>

namespace playertia::clock {

struct DateTime {
  std::uint16_t year{};
  std::uint8_t month{};
  std::uint8_t day{};
  std::uint8_t weekday{};
  std::uint8_t hour{};
  std::uint8_t minute{};
  std::uint8_t second{};
};

struct HandTarget {
  double angle_rad{};
  double angular_velocity_rad_s{};
};

using HandTargets = std::array<HandTarget, 3>;

struct HandFeel {
  float mass_kg{};
  float center_of_mass_m{};
  float inertia_kg_m2{};
  float damping_ratio{};
  float quadratic_damping_scale{};
  float coulomb_friction_fraction{};
};

using HandFeels = std::array<HandFeel, 3>;

enum class InteractionMode : std::uint8_t {
  Locked,
  Free,
  Returning,
};

[[nodiscard]] std::int64_t dateTimeToEpoch(
    const DateTime& value) noexcept;
[[nodiscard]] DateTime epochToDateTime(std::int64_t epoch) noexcept;
[[nodiscard]] HandTargets handTargets(double local_epoch) noexcept;
[[nodiscard]] HandFeels handFeels() noexcept;

[[nodiscard]] InteractionMode shortPress(
    InteractionMode current) noexcept;
[[nodiscard]] bool acceptsTimeTarget(InteractionMode mode) noexcept;
[[nodiscard]] bool acceptsPhysicalInput(InteractionMode mode) noexcept;
[[nodiscard]] bool acceptsTouch(InteractionMode mode) noexcept;

}  // namespace playertia::clock
