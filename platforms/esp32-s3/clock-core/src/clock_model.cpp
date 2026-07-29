#include "playertia/clock/clock_model.hpp"

#include <cmath>

namespace playertia::clock {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

std::int64_t daysFromCivil(int year,
                           const unsigned month,
                           const unsigned day) noexcept {
  year -= month <= 2U;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era =
      static_cast<unsigned>(year - era * 400);
  const unsigned adjusted_month =
      month > 2U ? month - 3U : month + 9U;
  const unsigned day_of_year =
      (153U * adjusted_month + 2U) / 5U + day - 1U;
  const unsigned day_of_era =
      year_of_era * 365U + year_of_era / 4U -
      year_of_era / 100U + day_of_year;
  return static_cast<std::int64_t>(era) * 146097LL +
         static_cast<std::int64_t>(day_of_era) - 719468LL;
}

}  // namespace

std::int64_t dateTimeToEpoch(const DateTime& value) noexcept {
  return daysFromCivil(value.year, value.month, value.day) * 86400LL +
         static_cast<std::int64_t>(value.hour) * 3600LL +
         static_cast<std::int64_t>(value.minute) * 60LL +
         static_cast<std::int64_t>(value.second);
}

DateTime epochToDateTime(std::int64_t epoch) noexcept {
  std::int64_t days = epoch / 86400LL;
  std::int64_t seconds = epoch % 86400LL;
  if (seconds < 0) {
    seconds += 86400LL;
    --days;
  }

  const std::int64_t shifted = days + 719468LL;
  const std::int64_t era =
      (shifted >= 0 ? shifted : shifted - 146096LL) / 146097LL;
  const unsigned day_of_era =
      static_cast<unsigned>(shifted - era * 146097LL);
  const unsigned year_of_era =
      (day_of_era - day_of_era / 1460U + day_of_era / 36524U -
       day_of_era / 146096U) /
      365U;
  int year =
      static_cast<int>(year_of_era) + static_cast<int>(era * 400LL);
  const unsigned day_of_year =
      day_of_era -
      (365U * year_of_era + year_of_era / 4U - year_of_era / 100U);
  const unsigned month_prime = (5U * day_of_year + 2U) / 153U;
  const unsigned day =
      day_of_year - (153U * month_prime + 2U) / 5U + 1U;
  const unsigned month =
      month_prime < 10U ? month_prime + 3U : month_prime - 9U;
  year += month <= 2U;

  DateTime result{};
  result.year = static_cast<std::uint16_t>(year);
  result.month = static_cast<std::uint8_t>(month);
  result.day = static_cast<std::uint8_t>(day);
  const std::int64_t weekday = (days + 4LL) % 7LL;
  result.weekday = static_cast<std::uint8_t>(
      weekday < 0 ? weekday + 7LL : weekday);
  result.hour = static_cast<std::uint8_t>(seconds / 3600LL);
  result.minute =
      static_cast<std::uint8_t>((seconds % 3600LL) / 60LL);
  result.second = static_cast<std::uint8_t>(seconds % 60LL);
  return result;
}

HandTargets handTargets(const double local_epoch) noexcept {
  double seconds_in_day = std::fmod(local_epoch, 86400.0);
  if (seconds_in_day < 0.0) {
    seconds_in_day += 86400.0;
  }
  const double seconds_in_hour =
      std::fmod(seconds_in_day, 3600.0);
  const double seconds_in_minute =
      std::fmod(seconds_in_day, 60.0);

  return {{
      {seconds_in_day * kTwoPi / 43200.0, kTwoPi / 43200.0},
      {seconds_in_hour * kTwoPi / 3600.0, kTwoPi / 3600.0},
      {seconds_in_minute * kTwoPi / 60.0, kTwoPi / 60.0},
  }};
}

HandFeels handFeels() noexcept {
  // Hour, minute, second. These are pivoted-body parameters: perceived
  // weight follows mass distribution and inertia, not mass alone.
  // The free-play profile deliberately uses three distinct damping levels.
  // Viscous damping shapes normal decay, quadratic damping tames violent
  // spins, and non-zero Coulomb friction guarantees a finite low-speed stop.
  return {{
      {0.00055F, 0.0048F, 7.0e-8F, 0.045F, 0.0050F, 0.00150F},
      {0.00032F, 0.0068F, 5.0e-8F, 0.024F, 0.0030F, 0.00080F},
      {0.00012F, 0.0078F, 2.0e-8F, 0.012F, 0.0020F, 0.00040F},
  }};
}

InteractionMode shortPress(const InteractionMode current) noexcept {
  return current == InteractionMode::Free
             ? InteractionMode::Returning
             : InteractionMode::Free;
}

bool acceptsTimeTarget(const InteractionMode mode) noexcept {
  return mode != InteractionMode::Free;
}

bool acceptsPhysicalInput(const InteractionMode mode) noexcept {
  return mode == InteractionMode::Free;
}

bool acceptsTouch(const InteractionMode mode) noexcept {
  return mode == InteractionMode::Free;
}

}  // namespace playertia::clock
