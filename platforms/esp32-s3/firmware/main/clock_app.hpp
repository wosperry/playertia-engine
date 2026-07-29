#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "board_services.hpp"
#include "dial/physics.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "lvgl.h"
#include "playertia/esp32_s3/app_shell.hpp"

namespace playertia::esp32_s3 {

class ClockApplication final : public Application {
 public:
  explicit ClockApplication(BoardServices& services) noexcept;

  [[nodiscard]] std::string_view id() const noexcept override;
  [[nodiscard]] std::string_view title() const noexcept override;
  [[nodiscard]] bool load(AppContext& context) noexcept override;
  void unload(AppContext& context) noexcept override;
  void update(AppContext& context, std::uint32_t elapsed_ms) noexcept override;
  void render(AppContext& context) noexcept override;
  [[nodiscard]] bool handleButton(AppContext& context,
                                  HardwareButton button,
                                  ButtonAction action) noexcept override;

 private:
  struct TouchInteraction {
    bool active{};
    bool release{};
    std::int8_t hand{-1};
    float angle{};
    float angular_velocity{};
    float previous_angle{};
    std::uint64_t previous_us{};
  };

  static void touchEvent(lv_event_t* event);
  void handleTouch(lv_event_code_t code, lv_point_t point) noexcept;
  void createFace() noexcept;
  void destroyFace() noexcept;
  void createLine(lv_obj_t*& object,
                  lv_color_t color,
                  std::int32_t width,
                  lv_opa_t opacity = LV_OPA_COVER) noexcept;
  void updateHandGeometry() noexcept;
  void updateLabels() noexcept;
  void stepPhysics() noexcept;
  [[nodiscard]] std::array<dial::TimeTarget, dial::kHandCount>
  timeTargets(double local_epoch, std::uint64_t timestamp_us) const noexcept;
  [[nodiscard]] std::int8_t nearestHand(float angle) const noexcept;

  BoardServices& services_;
  dial::PhysicalDialEngine engine_;
  MotionFrame last_motion_{};
  TouchInteraction touch_{};
  portMUX_TYPE touch_lock_ = portMUX_INITIALIZER_UNLOCKED;
  dial::EventType pending_event_{dial::EventType::ForceTimeTracking};
  bool event_pending_{};
  bool loaded_{};
  bool labels_initialized_{};
  bool hand_geometry_initialized_{};
  std::uint64_t physics_time_us_{};
  std::uint32_t physics_accumulator_ms_{};
  std::uint8_t displayed_hour_{0xFFU};
  std::uint8_t displayed_minute_{0xFFU};
  std::uint8_t displayed_second_{0xFFU};
  std::uint16_t displayed_year_{};
  std::uint8_t displayed_month_{};
  std::uint8_t displayed_day_{};
  dial::GlobalMode displayed_mode_{dial::GlobalMode::Degraded};

  lv_obj_t* root_{};
  lv_obj_t* mode_label_{};
  lv_obj_t* date_label_{};
  lv_obj_t* digital_label_{};
  lv_obj_t* hand_lines_[3]{};
  lv_obj_t* hand_shadows_[3]{};
  lv_obj_t* center_cap_{};
  std::array<std::array<lv_point_precise_t, 2>, 60> tick_points_{};
  std::array<lv_obj_t*, 60> ticks_{};
  std::array<std::array<lv_point_precise_t, 3>, 3> hand_points_{};
  std::array<std::array<lv_point_precise_t, 3>, 3> shadow_points_{};
};

}  // namespace playertia::esp32_s3
