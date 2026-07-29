#include <cstdint>
#include <string_view>

#include "board_services.hpp"
#include "bsp/esp-bsp.h"
#include "clock_app.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "playertia/esp32_s3/app_shell.hpp"

#ifndef PLAYERTIA_BUILD_EPOCH
#define PLAYERTIA_BUILD_EPOCH 0
#endif

namespace {

using playertia::esp32_s3::AppContext;
using playertia::esp32_s3::AppManager;
using playertia::esp32_s3::BoardServices;
using playertia::esp32_s3::ButtonAction;
using playertia::esp32_s3::Canvas;
using playertia::esp32_s3::ClockApplication;
using playertia::esp32_s3::Color;
using playertia::esp32_s3::HardwareButton;
using playertia::esp32_s3::PlaygroundDemoApp;

constexpr gpio_num_t kTopRightButton = GPIO_NUM_0;
constexpr std::uint32_t kRuntimePeriodMs = 5;
constexpr TickType_t kRuntimePeriodTicks =
    pdMS_TO_TICKS(kRuntimePeriodMs);
constexpr std::uint32_t kRenderPeriodUs = 16'667;
constexpr std::uint64_t kStatsPeriodUs = 10'000'000ULL;
constexpr TickType_t kDebounceTicks = pdMS_TO_TICKS(35);
constexpr TickType_t kLongPressTicks = pdMS_TO_TICKS(900);
constexpr std::uint32_t kDisplayLockTimeoutMs = 1000;
constexpr int kDisplayBrightnessPercent = 72;

constexpr char kTag[] = "playertia";

lv_color_t toLvColor(const Color color) noexcept {
  return lv_color_make(color.red, color.green, color.blue);
}

class LvglCanvas final : public Canvas {
 public:
  explicit LvglCanvas(lv_obj_t* screen) noexcept : screen_(screen) {}

  void clear(const Color color) noexcept override {
    lv_obj_clean(screen_);
    lv_obj_set_style_bg_color(screen_, toLvColor(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(screen_, LV_OBJ_FLAG_SCROLLABLE);
  }

  void fillRect(const std::int16_t x,
                const std::int16_t y,
                const std::int16_t width,
                const std::int16_t height,
                const Color color) noexcept override {
    lv_obj_t* object = lv_obj_create(screen_);
    lv_obj_remove_style_all(object);
    lv_obj_set_pos(object, x, y);
    lv_obj_set_size(object, width, height);
    lv_obj_set_style_bg_color(object, toLvColor(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(object, 4, LV_PART_MAIN);
    lv_obj_clear_flag(object, LV_OBJ_FLAG_SCROLLABLE);
  }

  void fillCircle(const std::int16_t center_x,
                  const std::int16_t center_y,
                  const std::int16_t radius,
                  const Color color) noexcept override {
    lv_obj_t* object = lv_obj_create(screen_);
    lv_obj_remove_style_all(object);
    lv_obj_set_pos(object, center_x - radius, center_y - radius);
    lv_obj_set_size(object, radius * 2, radius * 2);
    lv_obj_set_style_bg_color(object, toLvColor(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(object, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_clear_flag(object, LV_OBJ_FLAG_SCROLLABLE);
  }

  void drawText(const std::int16_t x,
                const std::int16_t y,
                const std::string_view text,
                const Color color) noexcept override {
    lv_obj_t* label = lv_label_create(screen_);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_color(label, toLvColor(color), LV_PART_MAIN);
    lv_label_set_text_fmt(
        label, "%.*s", static_cast<int>(text.size()), text.data());
  }

 private:
  lv_obj_t* screen_;
};

struct FirmwareRuntime {
  AppManager* manager;
};

void logActiveApplication(const AppManager& manager) {
  const auto* application = manager.activeApplication();
  if (application == nullptr) {
    ESP_LOGW(kTag, "No active application");
    return;
  }
  const std::string_view id = application->id();
  ESP_LOGI(kTag, "Active application: %.*s",
           static_cast<int>(id.size()), id.data());
}

void renderActiveApplication(AppManager& manager) {
  if (!bsp_display_lock(kDisplayLockTimeoutMs)) {
    ESP_LOGE(kTag, "LVGL lock timeout");
    return;
  }
  manager.render();
  bsp_display_unlock();
}

void dispatchButton(FirmwareRuntime& runtime,
                    const ButtonAction action) {
  if (!bsp_display_lock(kDisplayLockTimeoutMs)) {
    ESP_LOGE(kTag, "LVGL lock timeout during button action");
    return;
  }
  const auto result = runtime.manager->handleButton(
      HardwareButton::top_right, action);
  runtime.manager->render();
  bsp_display_unlock();
  ESP_LOGI(kTag, "Top-right action=%u result=%u",
           static_cast<unsigned>(action),
           static_cast<unsigned>(result));
  logActiveApplication(*runtime.manager);
}

void runtimeTask(void* argument) {
  auto* runtime = static_cast<FirmwareRuntime*>(argument);
  TickType_t next_wake = xTaskGetTickCount();
  int stable_level = gpio_get_level(kTopRightButton);
  int candidate_level = stable_level;
  TickType_t candidate_since = xTaskGetTickCount();
  TickType_t pressed_since = candidate_since;
  bool long_press_sent = false;
  std::uint32_t render_accumulator_us = 0;
  std::uint64_t stats_started_us = esp_timer_get_time();
  std::uint64_t update_total_us = 0;
  std::uint64_t render_total_us = 0;
  std::uint64_t busy_total_us = 0;
  std::uint32_t cycle_count = 0;
  std::uint32_t render_count = 0;
  std::uint32_t deadline_misses = 0;
  std::uint32_t maximum_cycle_us = 0;

  while (true) {
    xTaskDelayUntil(&next_wake, kRuntimePeriodTicks);
    const std::uint64_t cycle_started_us = esp_timer_get_time();
    runtime->manager->update(kRuntimePeriodMs);
    const std::uint64_t update_finished_us = esp_timer_get_time();
    update_total_us += update_finished_us - cycle_started_us;

    render_accumulator_us += kRuntimePeriodMs * 1000U;
    if (render_accumulator_us >= kRenderPeriodUs) {
      render_accumulator_us -= kRenderPeriodUs;
      const std::uint64_t render_started_us = esp_timer_get_time();
      renderActiveApplication(*runtime->manager);
      render_total_us += esp_timer_get_time() - render_started_us;
      ++render_count;
    }

    const int raw_level = gpio_get_level(kTopRightButton);
    const TickType_t now = xTaskGetTickCount();
    if (raw_level != candidate_level) {
      candidate_level = raw_level;
      candidate_since = now;
    } else if (candidate_level != stable_level &&
               now - candidate_since >= kDebounceTicks) {
      stable_level = candidate_level;
      if (stable_level == 0) {
        pressed_since = now;
        long_press_sent = false;
      } else if (!long_press_sent) {
        dispatchButton(*runtime, ButtonAction::released);
      }
    }

    if (stable_level == 0 && !long_press_sent &&
        now - pressed_since >= kLongPressTicks) {
      long_press_sent = true;
      dispatchButton(*runtime, ButtonAction::long_pressed);
    }

    const std::uint64_t cycle_finished_us = esp_timer_get_time();
    const std::uint32_t cycle_us = static_cast<std::uint32_t>(
        cycle_finished_us - cycle_started_us);
    busy_total_us += cycle_us;
    if (cycle_us > maximum_cycle_us) {
      maximum_cycle_us = cycle_us;
    }
    if (cycle_us > kRuntimePeriodMs * 1000U) {
      ++deadline_misses;
    }
    ++cycle_count;

    const std::uint64_t stats_elapsed_us =
        cycle_finished_us - stats_started_us;
    if (stats_elapsed_us >= kStatsPeriodUs) {
      const double busy_percent =
          100.0 * static_cast<double>(busy_total_us) /
          static_cast<double>(stats_elapsed_us);
      const double update_average_us =
          static_cast<double>(update_total_us) /
          static_cast<double>(cycle_count);
      const double render_average_us =
          render_count == 0U
              ? 0.0
              : static_cast<double>(render_total_us) /
                    static_cast<double>(render_count);
      const double measured_fps =
          static_cast<double>(render_count) * 1.0e6 /
          static_cast<double>(stats_elapsed_us);
      ESP_LOGI(kTag,
               "Runtime load=%.1f%% update_avg=%.1fus "
               "render_avg=%.1fus max_cycle=%uus misses=%u fps=%.1f",
               busy_percent, update_average_us, render_average_us,
               static_cast<unsigned>(maximum_cycle_us),
               static_cast<unsigned>(deadline_misses), measured_fps);
      stats_started_us = cycle_finished_us;
      update_total_us = 0;
      render_total_us = 0;
      busy_total_us = 0;
      cycle_count = 0;
      render_count = 0;
      deadline_misses = 0;
      maximum_cycle_us = 0;
    }
  }
}

void configureTopRightButton() {
  gpio_config_t configuration{};
  configuration.pin_bit_mask =
      1ULL << static_cast<unsigned>(kTopRightButton);
  configuration.mode = GPIO_MODE_INPUT;
  configuration.pull_up_en = GPIO_PULLUP_ENABLE;
  configuration.pull_down_en = GPIO_PULLDOWN_DISABLE;
  configuration.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&configuration));
}

lv_obj_t* activeScreen() {
  return lv_screen_active();
}

}  // namespace

extern "C" void app_main(void) {
  ESP_LOGI(kTag, "Playertia physical clock firmware starting");
  ESP_LOGI(kTag, "Short press=clock action, long press=switch app");

  lv_display_t* display = bsp_display_start();
  if (display == nullptr) {
    ESP_LOGE(kTag, "Waveshare BSP display initialization failed");
    return;
  }
  ESP_ERROR_CHECK(
      bsp_display_brightness_set(kDisplayBrightnessPercent));
  configureTopRightButton();

  static BoardServices services;
  (void)services.initialize(
      static_cast<std::int64_t>(PLAYERTIA_BUILD_EPOCH));

  static LvglCanvas canvas(activeScreen());
  static AppContext context{canvas, 368, 448};
  static ClockApplication clock_app(services);
  static PlaygroundDemoApp playground_app;
  static AppManager manager(context);
  static FirmwareRuntime runtime{&manager};

  ESP_ERROR_CHECK(manager.registerApplication(clock_app)
                      ? ESP_OK
                      : ESP_ERR_INVALID_STATE);
  ESP_ERROR_CHECK(manager.registerApplication(playground_app)
                      ? ESP_OK
                      : ESP_ERR_INVALID_STATE);

  if (!bsp_display_lock(kDisplayLockTimeoutMs)) {
    ESP_LOGE(kTag, "LVGL lock timeout during initial load");
    return;
  }
  const bool started = manager.start();
  manager.render();
  bsp_display_unlock();
  ESP_ERROR_CHECK(started ? ESP_OK : ESP_FAIL);
  logActiveApplication(manager);

  const BaseType_t task_result = xTaskCreate(
      runtimeTask, "playertia_runtime", 8192, &runtime, 6, nullptr);
  ESP_ERROR_CHECK(task_result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

  ESP_LOGI(kTag, "Ready. RTC retained by PCF85063A");
}
