#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace playertia::esp32_s3 {

struct Color {
  std::uint8_t red{};
  std::uint8_t green{};
  std::uint8_t blue{};
};

class Canvas {
 public:
  virtual ~Canvas() = default;

  virtual void clear(Color color) noexcept = 0;
  virtual void fillRect(std::int16_t x,
                        std::int16_t y,
                        std::int16_t width,
                        std::int16_t height,
                        Color color) noexcept = 0;
  virtual void fillCircle(std::int16_t center_x,
                          std::int16_t center_y,
                          std::int16_t radius,
                          Color color) noexcept = 0;
  virtual void drawText(std::int16_t x,
                        std::int16_t y,
                        std::string_view text,
                        Color color) noexcept = 0;
};

struct AppContext {
  Canvas& canvas;
  std::uint16_t display_width{368};
  std::uint16_t display_height{448};
};

enum class HardwareButton : std::uint8_t;
enum class ButtonAction : std::uint8_t;

class Application {
 public:
  virtual ~Application() = default;

  [[nodiscard]] virtual std::string_view id() const noexcept = 0;
  [[nodiscard]] virtual std::string_view title() const noexcept = 0;
  [[nodiscard]] virtual bool load(AppContext& context) noexcept = 0;
  virtual void unload(AppContext& context) noexcept = 0;
  virtual void update(AppContext& context, std::uint32_t elapsed_ms) noexcept = 0;
  virtual void render(AppContext& context) noexcept = 0;
  [[nodiscard]] virtual bool handleButton(AppContext& context,
                                          HardwareButton button,
                                          ButtonAction action) noexcept;
};

enum class HardwareButton : std::uint8_t {
  top_right,
  other,
};

enum class ButtonAction : std::uint8_t {
  pressed,
  released,
  long_pressed,
};

enum class SwitchResult : std::uint8_t {
  switched,
  application_handled,
  ignored,
  no_applications,
  single_application,
  load_failed_restored,
  load_failed_inactive,
};

class AppManager {
 public:
  static constexpr std::size_t max_applications = 8;
  static constexpr std::size_t no_active_application =
      static_cast<std::size_t>(-1);

  explicit AppManager(AppContext& context) noexcept;

  [[nodiscard]] bool registerApplication(Application& application) noexcept;
  [[nodiscard]] bool start(std::size_t initial_index = 0) noexcept;
  void stop() noexcept;

  [[nodiscard]] SwitchResult handleButton(HardwareButton button,
                                          ButtonAction action) noexcept;
  [[nodiscard]] SwitchResult switchToNext() noexcept;

  void update(std::uint32_t elapsed_ms) noexcept;
  void render() noexcept;

  [[nodiscard]] Application* activeApplication() noexcept;
  [[nodiscard]] const Application* activeApplication() const noexcept;
  [[nodiscard]] std::size_t activeIndex() const noexcept;
  [[nodiscard]] std::size_t applicationCount() const noexcept;

 private:
  AppContext& context_;
  std::array<Application*, max_applications> applications_{};
  std::size_t application_count_{0};
  std::size_t active_index_{no_active_application};
};

struct DemoLifecycleStats {
  std::uint32_t load_count{0};
  std::uint32_t unload_count{0};
  std::uint32_t update_count{0};
  std::uint32_t render_count{0};
};

class StatusDemoApp final : public Application {
 public:
  [[nodiscard]] std::string_view id() const noexcept override;
  [[nodiscard]] std::string_view title() const noexcept override;
  [[nodiscard]] bool load(AppContext& context) noexcept override;
  void unload(AppContext& context) noexcept override;
  void update(AppContext& context, std::uint32_t elapsed_ms) noexcept override;
  void render(AppContext& context) noexcept override;

  [[nodiscard]] const DemoLifecycleStats& stats() const noexcept;

 private:
  DemoLifecycleStats stats_{};
  std::uint32_t elapsed_ms_{0};
};

class PlaygroundDemoApp final : public Application {
 public:
  [[nodiscard]] std::string_view id() const noexcept override;
  [[nodiscard]] std::string_view title() const noexcept override;
  [[nodiscard]] bool load(AppContext& context) noexcept override;
  void unload(AppContext& context) noexcept override;
  void update(AppContext& context, std::uint32_t elapsed_ms) noexcept override;
  void render(AppContext& context) noexcept override;

  [[nodiscard]] const DemoLifecycleStats& stats() const noexcept;

 private:
  DemoLifecycleStats stats_{};
  std::uint32_t elapsed_ms_{0};
};

}  // namespace playertia::esp32_s3
