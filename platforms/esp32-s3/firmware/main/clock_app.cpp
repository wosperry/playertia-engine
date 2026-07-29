#include "clock_app.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"

namespace playertia::esp32_s3 {
namespace {

constexpr char kTag[] = "clock_app";
constexpr std::int32_t kCenterX = 184;
constexpr std::int32_t kCenterY = 214;
constexpr std::int32_t kDialRadius = 164;
constexpr std::int32_t kUtcOffsetSeconds = 8 * 3600;
constexpr float kPhysicsDt = 0.01F;
constexpr std::uint64_t kPhysicsDtUs = 10'000ULL;
constexpr float kReturnDurationSec = 0.28F;
constexpr float kGravityMps2 = 9.80665F;

lv_color_t color(const std::uint32_t value) noexcept {
  return lv_color_hex(value);
}

lv_point_precise_t pointOnDial(const float angle,
                               const float radius,
                               const float x_offset = 0.0F,
                               const float y_offset = 0.0F) noexcept {
  return {
      static_cast<lv_value_precise_t>(
          static_cast<float>(kCenterX) + std::sin(angle) * radius +
          x_offset),
      static_cast<lv_value_precise_t>(
          static_cast<float>(kCenterY) - std::cos(angle) * radius +
          y_offset),
  };
}

const char* modeText(const dial::GlobalMode mode) noexcept {
  switch (mode) {
    case dial::GlobalMode::TimeTracking:
      return "LOCKED";
    case dial::GlobalMode::PhysicsPlay:
      return "FREE";
    case dial::GlobalMode::AutoReturn:
      return "RETURN";
    case dial::GlobalMode::Degraded:
      return "SAFE";
  }
  return "SAFE";
}

}  // namespace

ClockApplication::ClockApplication(BoardServices& services) noexcept
    : services_(services) {}

std::string_view ClockApplication::id() const noexcept {
  return "playertia.clock";
}

std::string_view ClockApplication::title() const noexcept {
  return "Clock";
}

bool ClockApplication::load(AppContext&) noexcept {
  dial::EngineConfig config = dial::makeDefaultConfig();
  config.returnStrategy = dial::ReturnStrategy::Animation;
  config.hybridCaptureAngleRad = 0.16F;
  config.animationDurationSec = kReturnDurationSec;
  config.animationPreserveEntryVelocity = false;
  config.settleAngleRad = 0.018F;
  config.settleVelocityRadSec = 0.08F;
  for (auto& hand : config.hands) {
    hand.kind = dial::HandKind::Rigid;
  }
  const playertia::clock::HandFeels feels =
      playertia::clock::handFeels();
  for (std::size_t index = 0; index < config.hands.size(); ++index) {
    dial::HandDefinition& hand = config.hands[index];
    const playertia::clock::HandFeel& feel = feels[index];
    hand.massKg = feel.mass_kg;
    hand.centerOfMassAtZeroM =
        {0.0F, -feel.center_of_mass_m, 0.0F};
    hand.inertiaPivotKgM2 = feel.inertia_kg_m2;
    const float gravity_stiffness =
        feel.mass_kg * feel.center_of_mass_m * kGravityMps2;
    hand.viscousDampingNmSecPerRad =
        2.0F * feel.damping_ratio *
        std::sqrt(feel.inertia_kg_m2 * gravity_stiffness);
    hand.quadraticDampingNmSec2PerRad2 =
        hand.viscousDampingNmSecPerRad *
        feel.quadratic_damping_scale;
    hand.coulombFrictionNm =
        gravity_stiffness * feel.coulomb_friction_fraction;
    hand.timeKpNmPerRad = feel.inertia_kg_m2 * 64.0F;
    hand.timeKdNmSecPerRad =
        2.0F * 0.85F * feel.inertia_kg_m2 * 8.0F;
    hand.maxTimeTorqueNm = hand.timeKpNmPerRad * 4.0F;
    hand.touchKpNmPerRad = feel.inertia_kg_m2 * 400.0F;
    hand.touchKdNmSecPerRad =
        2.0F * 0.75F * feel.inertia_kg_m2 * 20.0F;
    hand.maxTouchTorqueNm = hand.touchKpNmPerRad;
    hand.maxRelativeKineticEnergyJ =
        0.5F * feel.inertia_kg_m2 *
        hand.maxAngularSpeedRadSec * hand.maxAngularSpeedRadSec;
  }
  const float stability =
      dial::maximumDimensionlessSubstep(config);
  ESP_LOGI(kTag, "Physics stability number %.4f (limit 0.2500)",
           static_cast<double>(stability));
  if (!engine_.configure(config)) {
    ESP_LOGE(kTag,
             "Physics profile rejected; using safe rigid defaults");
    dial::EngineConfig fallback = dial::makeDefaultConfig();
    fallback.returnStrategy = dial::ReturnStrategy::Animation;
    fallback.animationDurationSec = kReturnDurationSec;
    fallback.animationPreserveEntryVelocity = false;
    for (auto& hand : fallback.hands) {
      hand.kind = dial::HandKind::Rigid;
    }
    if (!engine_.configure(fallback)) {
      ESP_LOGE(kTag, "Safe physics configuration also rejected");
      return false;
    }
  }

  physics_time_us_ = esp_timer_get_time();
  const double local_epoch =
      services_.utcEpochSeconds() + kUtcOffsetSeconds;
  const auto targets = timeTargets(local_epoch, physics_time_us_);
  engine_.reset(
      {targets[0].angleUnwrappedRad, targets[1].angleUnwrappedRad,
       targets[2].angleUnwrappedRad},
      {targets[0].angularVelocityRadSec,
       targets[1].angularVelocityRadSec,
       targets[2].angularVelocityRadSec},
      {});

  createFace();
  loaded_ = root_ != nullptr;
  physics_accumulator_ms_ = 0;
  ESP_LOGI(kTag, "Clock loaded at local epoch %.3f", local_epoch);
  return loaded_;
}

void ClockApplication::unload(AppContext&) noexcept {
  destroyFace();
  loaded_ = false;
  portENTER_CRITICAL(&touch_lock_);
  touch_ = {};
  portEXIT_CRITICAL(&touch_lock_);
}

void ClockApplication::update(AppContext&,
                              const std::uint32_t elapsed_ms) noexcept {
  if (!loaded_) {
    return;
  }
  physics_accumulator_ms_ += elapsed_ms;
  while (physics_accumulator_ms_ >= 10U) {
    stepPhysics();
    physics_accumulator_ms_ -= 10U;
  }
}

void ClockApplication::render(AppContext&) noexcept {
  if (!loaded_) {
    return;
  }
  updateHandGeometry();
  updateLabels();
}

bool ClockApplication::handleButton(AppContext&,
                                    const HardwareButton button,
                                    const ButtonAction action) noexcept {
  if (button != HardwareButton::top_right ||
      action != ButtonAction::released) {
    return false;
  }
  const dial::GlobalMode mode = engine_.snapshot().globalMode;
  pending_event_ =
      mode == dial::GlobalMode::PhysicsPlay
          ? dial::EventType::BeginReturn
          : dial::EventType::EnterPhysics;
  event_pending_ = true;
  ESP_LOGI(kTag, "Short press: %s",
           pending_event_ == dial::EventType::EnterPhysics
               ? "unlock hands"
               : "return to time");
  return true;
}

void ClockApplication::createLine(lv_obj_t*& object,
                                  const lv_color_t line_color,
                                  const std::int32_t width,
                                  const lv_opa_t opacity) noexcept {
  object = lv_line_create(root_);
  lv_obj_remove_style_all(object);
  lv_obj_set_style_line_color(object, line_color, LV_PART_MAIN);
  lv_obj_set_style_line_width(object, width, LV_PART_MAIN);
  lv_obj_set_style_line_rounded(object, true, LV_PART_MAIN);
  lv_obj_set_style_line_opa(object, opacity, LV_PART_MAIN);
}

void ClockApplication::createFace() noexcept {
  lv_obj_t* screen = lv_screen_active();
  lv_obj_clean(screen);
  lv_obj_set_style_bg_color(screen, color(0x07090C), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  root_ = lv_obj_create(screen);
  lv_obj_remove_style_all(root_);
  lv_obj_set_size(root_, LV_PCT(100), LV_PCT(100));
  lv_obj_set_pos(root_, 0, 0);
  lv_obj_add_flag(root_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(root_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(root_, touchEvent, LV_EVENT_ALL, this);

  lv_obj_t* bezel = lv_obj_create(root_);
  lv_obj_remove_style_all(bezel);
  lv_obj_set_pos(bezel, kCenterX - kDialRadius, kCenterY - kDialRadius);
  lv_obj_set_size(bezel, kDialRadius * 2, kDialRadius * 2);
  lv_obj_set_style_radius(bezel, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bezel, color(0x0D1116), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bezel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(bezel, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(bezel, color(0x3B4650), LV_PART_MAIN);
  lv_obj_clear_flag(bezel, LV_OBJ_FLAG_CLICKABLE);

  for (std::size_t index = 0; index < ticks_.size(); ++index) {
    const float angle =
        dial::kTwoPi * static_cast<float>(index) / 60.0F;
    const bool major = index % 5U == 0U;
    tick_points_[index][0] =
        pointOnDial(angle, major ? 139.0F : 147.0F);
    tick_points_[index][1] = pointOnDial(angle, 154.0F);
    createLine(ticks_[index],
               major ? color(0xC8D0D7) : color(0x4A545D),
               major ? 3 : 1,
               major ? LV_OPA_COVER : LV_OPA_70);
    lv_line_set_points(ticks_[index], tick_points_[index].data(), 2);
  }

  constexpr std::array<const char*, 4> numerals{"12", "3", "6", "9"};
  constexpr std::array<std::array<std::int32_t, 2>, 4> numeral_positions{{
      {174, 72}, {321, 207}, {179, 340}, {35, 207},
  }};
  for (std::size_t index = 0; index < numerals.size(); ++index) {
    lv_obj_t* label = lv_label_create(root_);
    lv_label_set_text(label, numerals[index]);
    lv_obj_set_pos(label, numeral_positions[index][0],
                   numeral_positions[index][1]);
    lv_obj_set_style_text_color(label, color(0xAEB8C1), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14,
                               LV_PART_MAIN);
  }

  lv_obj_t* brand = lv_label_create(root_);
  lv_label_set_text(brand, "PLAYERTIA");
  lv_obj_set_style_text_color(brand, color(0x65717B), LV_PART_MAIN);
  lv_obj_set_style_text_font(brand, &lv_font_montserrat_14,
                             LV_PART_MAIN);
  lv_obj_align(brand, LV_ALIGN_TOP_MID, 0, 18);

  digital_label_ = lv_label_create(root_);
  lv_obj_set_style_text_color(digital_label_, color(0x87939D),
                              LV_PART_MAIN);
  lv_obj_set_style_text_font(digital_label_, &lv_font_montserrat_14,
                             LV_PART_MAIN);
  lv_obj_align(digital_label_, LV_ALIGN_CENTER, 0, 65);

  mode_label_ = lv_label_create(root_);
  lv_obj_set_style_text_color(mode_label_, color(0x4EB6D9),
                              LV_PART_MAIN);
  lv_obj_set_style_text_font(mode_label_, &lv_font_montserrat_14,
                             LV_PART_MAIN);
  lv_obj_align(mode_label_, LV_ALIGN_CENTER, 0, 91);

  date_label_ = lv_label_create(root_);
  lv_obj_set_style_text_color(date_label_, color(0x65717B),
                              LV_PART_MAIN);
  lv_obj_set_style_text_font(date_label_, &lv_font_montserrat_14,
                             LV_PART_MAIN);
  lv_obj_align(date_label_, LV_ALIGN_BOTTOM_MID, 0, -20);

  constexpr std::array<std::uint32_t, 3> hand_colors{
      0xD7DEE4, 0xF0F3F5, 0x4EB6D9};
  constexpr std::array<std::int32_t, 3> hand_widths{8, 5, 2};
  for (std::size_t index = 0; index < 3; ++index) {
    createLine(hand_shadows_[index], color(0x010203),
               hand_widths[index] + 3, LV_OPA_60);
    createLine(hand_lines_[index], color(hand_colors[index]),
               hand_widths[index]);
  }
  // Establish the z-order once. Reordering these objects on every frame
  // invalidates most of the dial and makes the QSPI display repaint static
  // ticks and labels unnecessarily.
  for (lv_obj_t* hand : hand_lines_) {
    lv_obj_move_foreground(hand);
  }

  center_cap_ = lv_obj_create(root_);
  lv_obj_remove_style_all(center_cap_);
  lv_obj_set_size(center_cap_, 17, 17);
  lv_obj_set_pos(center_cap_, kCenterX - 8, kCenterY - 8);
  lv_obj_set_style_radius(center_cap_, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(center_cap_, color(0xD7DEE4), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(center_cap_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(center_cap_, 4, LV_PART_MAIN);
  lv_obj_set_style_border_color(center_cap_, color(0x303940),
                                LV_PART_MAIN);

  labels_initialized_ = false;
  updateHandGeometry();
  updateLabels();
}

void ClockApplication::destroyFace() noexcept {
  if (root_ != nullptr) {
    lv_obj_delete(root_);
  }
  root_ = nullptr;
  mode_label_ = nullptr;
  date_label_ = nullptr;
  digital_label_ = nullptr;
  center_cap_ = nullptr;
  hand_geometry_initialized_ = false;
  ticks_.fill(nullptr);
  std::fill(std::begin(hand_lines_), std::end(hand_lines_), nullptr);
  std::fill(std::begin(hand_shadows_), std::end(hand_shadows_), nullptr);
}

void ClockApplication::updateHandGeometry() noexcept {
  const dial::EngineSnapshot& snapshot = engine_.snapshot();
  constexpr std::array<float, 3> lengths{86.0F, 126.0F, 145.0F};
  constexpr std::array<float, 3> tails{8.0F, 10.0F, 24.0F};
  constexpr std::int32_t kDirtyMargin = 8;
  std::array<std::array<lv_point_precise_t, 3>, 3> next_points =
      hand_points_;
  std::array<std::array<lv_point_precise_t, 3>, 3> next_shadow_points =
      shadow_points_;
  std::array<bool, 3> changed{};
  lv_area_t dirty{};
  bool has_dirty = false;

  const auto include_segment =
      [&dirty, &has_dirty](const lv_point_precise_t first,
                           const lv_point_precise_t second) {
        const lv_area_t area{
            static_cast<std::int32_t>(
                std::min(first.x, second.x) - kDirtyMargin),
            static_cast<std::int32_t>(
                std::min(first.y, second.y) - kDirtyMargin),
            static_cast<std::int32_t>(
                std::max(first.x, second.x) + kDirtyMargin),
            static_cast<std::int32_t>(
                std::max(first.y, second.y) + kDirtyMargin),
        };
        if (!has_dirty) {
          dirty = area;
          has_dirty = true;
          return;
        }
        dirty.x1 = std::min(dirty.x1, area.x1);
        dirty.y1 = std::min(dirty.y1, area.y1);
        dirty.x2 = std::max(dirty.x2, area.x2);
        dirty.y2 = std::max(dirty.y2, area.y2);
      };

  for (std::size_t index = 0; index < 3; ++index) {
    const float angle =
        static_cast<float>(snapshot.hands[index].angleUnwrappedRad);

    next_points[index][0] =
        pointOnDial(angle + dial::kPi, tails[index]);
    next_points[index][1] = pointOnDial(angle, lengths[index]);
    for (std::size_t point = 0; point < 2; ++point) {
      next_shadow_points[index][point] = {
          static_cast<lv_value_precise_t>(
              next_points[index][point].x + 2),
          static_cast<lv_value_precise_t>(
              next_points[index][point].y + 3)};
    }
    changed[index] =
        next_points[index][0].x != hand_points_[index][0].x ||
        next_points[index][0].y != hand_points_[index][0].y ||
        next_points[index][1].x != hand_points_[index][1].x ||
        next_points[index][1].y != hand_points_[index][1].y;
    if (!changed[index]) {
      continue;
    }
    if (hand_geometry_initialized_) {
      include_segment(hand_points_[index][0], hand_points_[index][1]);
    }
    include_segment(next_points[index][0], next_points[index][1]);
  }

  if (!has_dirty) {
    return;
  }

  // Treat the six LVGL line objects as one visual transaction. Without this,
  // each old/new line rectangle can be flushed separately over QSPI, briefly
  // exposing a half-updated overlap or a one-frame flash on the first unlock.
  lv_display_t* display = lv_obj_get_display(root_);
  lv_display_enable_invalidation(display, false);
  for (std::size_t index = 0; index < 3; ++index) {
    if (!changed[index]) {
      continue;
    }
    hand_points_[index] = next_points[index];
    shadow_points_[index] = next_shadow_points[index];
    lv_line_set_points(hand_shadows_[index],
                       shadow_points_[index].data(), 2);
    lv_line_set_points(hand_lines_[index],
                       hand_points_[index].data(), 2);
  }
  lv_display_enable_invalidation(display, true);
  (void)lv_obj_invalidate_area(root_, &dirty);
  hand_geometry_initialized_ = true;
}

void ClockApplication::updateLabels() noexcept {
  const UtcDateTime local =
      services_.localDateTime(kUtcOffsetSeconds);
  char text[32]{};
  if (!labels_initialized_ || local.hour != displayed_hour_ ||
      local.minute != displayed_minute_ ||
      local.second != displayed_second_) {
    std::snprintf(text, sizeof(text), "%02u:%02u:%02u",
                  static_cast<unsigned>(local.hour),
                  static_cast<unsigned>(local.minute),
                  static_cast<unsigned>(local.second));
    lv_label_set_text(digital_label_, text);
    displayed_hour_ = local.hour;
    displayed_minute_ = local.minute;
    displayed_second_ = local.second;
  }
  if (!labels_initialized_ || local.year != displayed_year_ ||
      local.month != displayed_month_ || local.day != displayed_day_) {
    std::snprintf(text, sizeof(text), "%04u.%02u.%02u",
                  static_cast<unsigned>(local.year),
                  static_cast<unsigned>(local.month),
                  static_cast<unsigned>(local.day));
    lv_label_set_text(date_label_, text);
    displayed_year_ = local.year;
    displayed_month_ = local.month;
    displayed_day_ = local.day;
  }
  const dial::GlobalMode mode = engine_.snapshot().globalMode;
  if (!labels_initialized_ || mode != displayed_mode_) {
    lv_label_set_text(mode_label_, modeText(mode));
    displayed_mode_ = mode;
  }
  labels_initialized_ = true;
}

std::array<dial::TimeTarget, dial::kHandCount>
ClockApplication::timeTargets(const double local_epoch,
                              const std::uint64_t timestamp_us) const noexcept {
  const playertia::clock::HandTargets clock_targets =
      playertia::clock::handTargets(local_epoch);
  std::array<dial::TimeTarget, dial::kHandCount> targets{};
  for (std::size_t index = 0; index < targets.size(); ++index) {
    targets[index] = {
        static_cast<float>(clock_targets[index].angle_rad),
        static_cast<float>(
            clock_targets[index].angular_velocity_rad_s),
        0.0F,
        timestamp_us,
        true,
    };
  }
  return targets;
}

void ClockApplication::stepPhysics() noexcept {
  const dial::GlobalMode mode_before_step =
      engine_.snapshot().globalMode;
  const bool begins_return_this_step =
      event_pending_ &&
      pending_event_ == dial::EventType::BeginReturn;
  const bool physical_inputs_enabled =
      mode_before_step == dial::GlobalMode::PhysicsPlay &&
      !begins_return_this_step;
  const MotionFrame previous_motion = last_motion_;
  MotionFrame motion = services_.sampleMotion();
  if (motion.valid) {
    last_motion_ = motion;
  } else {
    motion = previous_motion;
  }
  if (!motion.valid) {
    motion.gravity_body_mps2 = {0.0F, 0.0F, -9.80665F};
    motion.effective_acceleration_body_mps2 =
        motion.gravity_body_mps2;
  }

  dial::BaseInterval interval{};
  interval.startUs = physics_time_us_;
  interval.endUs = physics_time_us_ + kPhysicsDtUs;
  const dial::Vec3f old_omega =
      physical_inputs_enabled && previous_motion.valid
          ? previous_motion.angular_velocity_body_rad_s
          : dial::Vec3f{};
  const dial::Vec3f new_omega =
      physical_inputs_enabled && motion.valid
          ? motion.angular_velocity_body_rad_s
          : old_omega;
  for (std::size_t quarter = 0; quarter < dial::kBaseQuarterCount;
       ++quarter) {
    dial::BaseSubInterval& sub = interval.quarter[quarter];
    sub.startUs = interval.startUs + quarter * (kPhysicsDtUs / 4U);
    sub.endUs = sub.startUs + kPhysicsDtUs / 4U;
    const float t0 = static_cast<float>(quarter) / 4.0F;
    const float t1 = static_cast<float>(quarter + 1U) / 4.0F;
    sub.omegaBodyAtStartRadSec =
        old_omega + (new_omega - old_omega) * t0;
    sub.omegaBodyAtEndRadSec =
        old_omega + (new_omega - old_omega) * t1;
    sub.gravityBodyAtEndMps2 =
        physical_inputs_enabled
            ? motion.gravity_body_mps2
            : dial::Vec3f{};
    sub.effectiveAccelImpulseMps =
        physical_inputs_enabled
            ? motion.effective_acceleration_body_mps2 *
                  (kPhysicsDt / 4.0F)
            : dial::Vec3f{};
    const dial::Vec3f omega =
        (sub.omegaBodyAtStartRadSec + sub.omegaBodyAtEndRadSec) * 0.5F;
    const float dt = kPhysicsDt / 4.0F;
    const float values[3]{omega.x, omega.y, omega.z};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        sub.omegaOuterProductIntegral.v[row][column] =
            values[row] * values[column] * dt;
      }
    }
    sub.qualityFlags =
        motion.quality_flags & ~dial::QualitySaturated;
  }
  interval.qualityFlags =
      motion.quality_flags & ~dial::QualitySaturated;
  if ((motion.quality_flags & dial::QualitySaturated) != 0U) {
    interval.qualityFlags |= dial::QualityDegraded;
  }

  dial::StepInput input{};
  input.timeTargets = timeTargets(
      services_.utcEpochSeconds() + kUtcOffsetSeconds, interval.endUs);
  // A manual unlock is a latched product state. Keep the generic engine's
  // inactivity timer from entering AutoReturn until the user explicitly
  // presses the button again.
  if (mode_before_step == dial::GlobalMode::PhysicsPlay) {
    input.intentionalInteraction = true;
  }
  if (event_pending_) {
    input.events[0] = {pending_event_, 0U, 0.0F, interval.startUs};
    input.eventCount = 1;
    input.intentionalInteraction = true;
    if (pending_event_ == dial::EventType::EnterPhysics) {
      ESP_LOGI(kTag, "Unlock gravity dial=(%.3f, %.3f, %.3f)",
               static_cast<double>(motion.gravity_body_mps2.x),
               static_cast<double>(motion.gravity_body_mps2.y),
               static_cast<double>(motion.gravity_body_mps2.z));
    }
    event_pending_ = false;
  }

  TouchInteraction touch{};
  portENTER_CRITICAL(&touch_lock_);
  touch = touch_;
  touch_.release = false;
  portEXIT_CRITICAL(&touch_lock_);
  if (physical_inputs_enabled && touch.hand >= 0 &&
      static_cast<std::size_t>(touch.hand) < dial::kHandCount) {
    auto& target = input.touchTargets[static_cast<std::size_t>(touch.hand)];
    target.active = touch.active;
    target.release = touch.release;
    target.angleUnwrappedRad = touch.angle;
    target.angularVelocityRadSec = touch.angular_velocity;
    target.timestampUs = interval.endUs;
    input.intentionalInteraction = touch.active || touch.release;
  } else if (!physical_inputs_enabled) {
    portENTER_CRITICAL(&touch_lock_);
    touch_ = {};
    portEXIT_CRITICAL(&touch_lock_);
  }

  if (!engine_.step(interval, input)) {
    ESP_LOGW(kTag, "Physics step rejected, error=%u",
             static_cast<unsigned>(engine_.snapshot().lastError));
  }
  physics_time_us_ = interval.endUs;
}

std::int8_t ClockApplication::nearestHand(const float angle) const noexcept {
  const dial::EngineSnapshot& snapshot = engine_.snapshot();
  std::int8_t nearest = 0;
  float minimum = dial::kPi;
  for (std::size_t index = 0; index < dial::kHandCount; ++index) {
    const float hand_angle =
        static_cast<float>(snapshot.hands[index].angleUnwrappedRad);
    const float distance =
        std::abs(dial::shortestAngle(hand_angle, angle));
    if (distance < minimum) {
      minimum = distance;
      nearest = static_cast<std::int8_t>(index);
    }
  }
  return nearest;
}

void ClockApplication::touchEvent(lv_event_t* event) {
  auto* application =
      static_cast<ClockApplication*>(lv_event_get_user_data(event));
  lv_indev_t* input = lv_indev_active();
  if (application == nullptr || input == nullptr) {
    return;
  }
  lv_point_t point{};
  lv_indev_get_point(input, &point);
  application->handleTouch(lv_event_get_code(event), point);
}

void ClockApplication::handleTouch(const lv_event_code_t code,
                                   const lv_point_t point) noexcept {
  if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING &&
      code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) {
    return;
  }
  const float x = static_cast<float>(point.x - kCenterX);
  const float y = static_cast<float>(point.y - kCenterY);
  const float radius = std::sqrt(x * x + y * y);
  const float angle = std::atan2(x, -y);
  const std::uint64_t now = esp_timer_get_time();

  portENTER_CRITICAL(&touch_lock_);
  if (code == LV_EVENT_PRESSED && radius > 34.0F &&
      radius < static_cast<float>(kDialRadius)) {
    touch_.active = true;
    touch_.release = false;
    touch_.hand = nearestHand(angle);
    touch_.angle = angle;
    touch_.previous_angle = angle;
    touch_.previous_us = now;
    touch_.angular_velocity = 0.0F;
  } else if (code == LV_EVENT_PRESSING && touch_.active) {
    const float dt =
        std::max(static_cast<float>(now - touch_.previous_us) * 1.0e-6F,
                 0.001F);
    const float delta =
        dial::shortestAngle(touch_.previous_angle, angle);
    touch_.angle += delta;
    touch_.angular_velocity =
        std::clamp(delta / dt, -40.0F, 40.0F);
    touch_.previous_angle = angle;
    touch_.previous_us = now;
  } else if ((code == LV_EVENT_RELEASED ||
              code == LV_EVENT_PRESS_LOST) &&
             touch_.active) {
    touch_.active = false;
    touch_.release = true;
  }
  portEXIT_CRITICAL(&touch_lock_);
}

}  // namespace playertia::esp32_s3
