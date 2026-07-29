#include "playertia/esp32_s3/app_shell.hpp"

#include <cassert>
#include <cstdint>
#include <string_view>

namespace {

class CountingCanvas final : public playertia::esp32_s3::Canvas {
 public:
  void clear(playertia::esp32_s3::Color) noexcept override {
    ++clear_count;
  }

  void fillRect(std::int16_t,
                std::int16_t,
                std::int16_t,
                std::int16_t,
                playertia::esp32_s3::Color) noexcept override {
    ++shape_count;
  }

  void fillCircle(std::int16_t,
                  std::int16_t,
                  std::int16_t,
                  playertia::esp32_s3::Color) noexcept override {
    ++shape_count;
  }

  void drawText(std::int16_t,
                std::int16_t,
                std::string_view,
                playertia::esp32_s3::Color) noexcept override {
    ++text_count;
  }

  std::uint32_t clear_count{0};
  std::uint32_t shape_count{0};
  std::uint32_t text_count{0};
};

class FailingApp final : public playertia::esp32_s3::Application {
 public:
  [[nodiscard]] std::string_view id() const noexcept override {
    return "test.failing";
  }

  [[nodiscard]] std::string_view title() const noexcept override {
    return "Failing";
  }

  [[nodiscard]] bool load(playertia::esp32_s3::AppContext&) noexcept override {
    ++load_count;
    return false;
  }

  void unload(playertia::esp32_s3::AppContext&) noexcept override {}

  void update(playertia::esp32_s3::AppContext&,
              std::uint32_t) noexcept override {}

  void render(playertia::esp32_s3::AppContext&) noexcept override {}

  std::uint32_t load_count{0};
};

class InteractiveApp final : public playertia::esp32_s3::Application {
 public:
  [[nodiscard]] std::string_view id() const noexcept override {
    return "test.interactive";
  }
  [[nodiscard]] std::string_view title() const noexcept override {
    return "Interactive";
  }
  [[nodiscard]] bool load(playertia::esp32_s3::AppContext&) noexcept override {
    return true;
  }
  void unload(playertia::esp32_s3::AppContext&) noexcept override {}
  void update(playertia::esp32_s3::AppContext&, std::uint32_t) noexcept override {}
  void render(playertia::esp32_s3::AppContext&) noexcept override {}
  [[nodiscard]] bool handleButton(
      playertia::esp32_s3::AppContext&,
      playertia::esp32_s3::HardwareButton button,
      playertia::esp32_s3::ButtonAction action) noexcept override {
    if (button == playertia::esp32_s3::HardwareButton::top_right &&
        action == playertia::esp32_s3::ButtonAction::released) {
      ++short_press_count;
      return true;
    }
    return false;
  }

  std::uint32_t short_press_count{};
};

}  // namespace

int main() {
  using namespace playertia::esp32_s3;

  CountingCanvas canvas;
  AppContext context{canvas};
  StatusDemoApp status;
  PlaygroundDemoApp playground;
  AppManager manager(context);

  assert(manager.registerApplication(status));
  assert(manager.registerApplication(playground));
  assert(!manager.registerApplication(status));
  assert(manager.applicationCount() == 2);

  assert(manager.start());
  assert(manager.activeApplication() == &status);
  assert(status.stats().load_count == 1);

  manager.update(16);
  manager.render();
  assert(status.stats().update_count == 1);
  assert(status.stats().render_count == 1);
  assert(canvas.clear_count == 1);

  assert(manager.handleButton(HardwareButton::other, ButtonAction::pressed) ==
         SwitchResult::ignored);
  assert(manager.handleButton(HardwareButton::top_right,
                              ButtonAction::released) ==
         SwitchResult::ignored);
  assert(manager.activeApplication() == &status);

  assert(manager.handleButton(HardwareButton::top_right,
                              ButtonAction::long_pressed) ==
         SwitchResult::switched);
  assert(status.stats().unload_count == 1);
  assert(playground.stats().load_count == 1);
  assert(manager.activeApplication() == &playground);

  manager.update(16);
  manager.render();
  assert(playground.stats().update_count == 1);
  assert(playground.stats().render_count == 1);

  assert(manager.switchToNext() == SwitchResult::switched);
  assert(playground.stats().unload_count == 1);
  assert(status.stats().load_count == 2);
  assert(manager.activeApplication() == &status);

  manager.stop();
  assert(status.stats().unload_count == 2);
  assert(manager.activeApplication() == nullptr);
  assert(manager.activeIndex() == AppManager::no_active_application);

  StatusDemoApp recoverable_status;
  FailingApp failing;
  AppManager recovery_manager(context);
  assert(recovery_manager.registerApplication(recoverable_status));
  assert(recovery_manager.registerApplication(failing));
  assert(recovery_manager.start());
  assert(recovery_manager.switchToNext() ==
         SwitchResult::load_failed_restored);
  assert(failing.load_count == 1);
  assert(recoverable_status.stats().unload_count == 1);
  assert(recoverable_status.stats().load_count == 2);
  assert(recovery_manager.activeApplication() == &recoverable_status);

  InteractiveApp interactive;
  StatusDemoApp secondary;
  AppManager interaction_manager(context);
  assert(interaction_manager.registerApplication(interactive));
  assert(interaction_manager.registerApplication(secondary));
  assert(interaction_manager.start());
  assert(interaction_manager.handleButton(HardwareButton::top_right,
                                          ButtonAction::released) ==
         SwitchResult::application_handled);
  assert(interactive.short_press_count == 1);
  assert(interaction_manager.activeApplication() == &interactive);
  assert(interaction_manager.handleButton(HardwareButton::top_right,
                                          ButtonAction::long_pressed) ==
         SwitchResult::switched);
  assert(interaction_manager.activeApplication() == &secondary);
}
