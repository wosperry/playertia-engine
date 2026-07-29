#include "playertia/clock/clock_model.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool close(const double left,
           const double right,
           const double tolerance = 1.0e-12) {
  return std::abs(left - right) <= tolerance;
}

void expectDate(const playertia::clock::DateTime value,
                const std::uint16_t year,
                const std::uint8_t month,
                const std::uint8_t day,
                const std::uint8_t hour,
                const std::uint8_t minute,
                const std::uint8_t second) {
  assert(value.year == year);
  assert(value.month == month);
  assert(value.day == day);
  assert(value.hour == hour);
  assert(value.minute == minute);
  assert(value.second == second);
}

}  // namespace

int main() {
  using namespace playertia::clock;

  const DateTime unix_epoch{1970, 1, 1, 4, 0, 0, 0};
  assert(dateTimeToEpoch(unix_epoch) == 0);
  expectDate(epochToDateTime(0), 1970, 1, 1, 0, 0, 0);

  const DateTime leap_day{2024, 2, 29, 4, 23, 59, 59};
  expectDate(epochToDateTime(dateTimeToEpoch(leap_day)),
             2024, 2, 29, 23, 59, 59);
  const DateTime century_leap{2000, 2, 29, 2, 12, 34, 56};
  expectDate(epochToDateTime(dateTimeToEpoch(century_leap)),
             2000, 2, 29, 12, 34, 56);

  const HandTargets midnight = handTargets(0.0);
  assert(close(midnight[0].angle_rad, 0.0));
  assert(close(midnight[1].angle_rad, 0.0));
  assert(close(midnight[2].angle_rad, 0.0));

  const double three_fifteen_thirty =
      3.0 * 3600.0 + 15.0 * 60.0 + 30.0;
  const HandTargets quarter = handTargets(three_fifteen_thirty);
  assert(close(quarter[0].angle_rad,
               three_fifteen_thirty * 2.0 * kPi / 43200.0));
  assert(close(quarter[1].angle_rad, 15.5 * 2.0 * kPi / 60.0));
  assert(close(quarter[2].angle_rad, kPi));

  const HandTargets next_day = handTargets(86400.0);
  for (std::size_t index = 0; index < next_day.size(); ++index) {
    assert(close(next_day[index].angle_rad, midnight[index].angle_rad));
    assert(next_day[index].angular_velocity_rad_s > 0.0);
  }

  const HandFeels feel = handFeels();
  assert(feel[0].mass_kg > feel[1].mass_kg);
  assert(feel[1].mass_kg > feel[2].mass_kg);
  assert(feel[0].inertia_kg_m2 > feel[1].inertia_kg_m2);
  assert(feel[1].inertia_kg_m2 > feel[2].inertia_kg_m2);
  assert(feel[0].damping_ratio > feel[1].damping_ratio);
  assert(feel[1].damping_ratio > feel[2].damping_ratio);
  for (const HandFeel& hand : feel) {
    assert(hand.inertia_kg_m2 >=
           hand.mass_kg * hand.center_of_mass_m *
               hand.center_of_mass_m);
  }
  constexpr float gravity = 9.80665F;
  const auto gravity_acceleration = [gravity](const HandFeel& hand) {
    return hand.mass_kg * hand.center_of_mass_m * gravity /
           hand.inertia_kg_m2;
  };
  const auto velocity_half_life = [gravity](const HandFeel& hand) {
    const float gravity_stiffness =
        hand.mass_kg * hand.center_of_mass_m * gravity;
    const float viscous_damping =
        2.0F * hand.damping_ratio *
        std::sqrt(hand.inertia_kg_m2 * gravity_stiffness);
    return std::log(2.0F) * hand.inertia_kg_m2 / viscous_damping;
  };
  assert(gravity_acceleration(feel[0]) <
         gravity_acceleration(feel[1]));
  assert(gravity_acceleration(feel[1]) <
         gravity_acceleration(feel[2]));
  assert(velocity_half_life(feel[0]) > 0.30F);
  assert(velocity_half_life(feel[0]) < 0.55F);
  assert(velocity_half_life(feel[0]) <
         velocity_half_life(feel[1]));
  assert(velocity_half_life(feel[1]) <
         velocity_half_life(feel[2]));
  assert(velocity_half_life(feel[2]) > 1.0F);
  assert(velocity_half_life(feel[2]) < 1.6F);
  for (const HandFeel& hand : feel) {
    assert(hand.coulomb_friction_fraction > 0.0F);
    assert(hand.quadratic_damping_scale > 0.0F);
  }

  assert(shortPress(InteractionMode::Locked) == InteractionMode::Free);
  assert(shortPress(InteractionMode::Free) ==
         InteractionMode::Returning);
  assert(shortPress(InteractionMode::Returning) ==
         InteractionMode::Free);
  assert(acceptsTimeTarget(InteractionMode::Locked));
  assert(!acceptsTimeTarget(InteractionMode::Free));
  assert(acceptsTimeTarget(InteractionMode::Returning));
  assert(!acceptsPhysicalInput(InteractionMode::Locked));
  assert(acceptsPhysicalInput(InteractionMode::Free));
  assert(!acceptsPhysicalInput(InteractionMode::Returning));
  assert(!acceptsTouch(InteractionMode::Locked));
  assert(acceptsTouch(InteractionMode::Free));
  assert(!acceptsTouch(InteractionMode::Returning));
}
