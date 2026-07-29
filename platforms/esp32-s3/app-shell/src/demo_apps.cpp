#include "playertia/esp32_s3/app_shell.hpp"

namespace playertia::esp32_s3 {
namespace {

constexpr Color kWhite{245, 247, 250};
constexpr Color kMuted{155, 170, 190};
constexpr Color kStatusBackground{9, 18, 34};
constexpr Color kStatusAccent{70, 135, 255};
constexpr Color kPlaygroundBackground{20, 12, 30};
constexpr Color kPlaygroundAccent{255, 104, 145};
constexpr Color kPlaygroundSecondary{121, 235, 192};

}  // namespace

std::string_view StatusDemoApp::id() const noexcept {
  return "demo.status";
}

std::string_view StatusDemoApp::title() const noexcept {
  return "Status";
}

bool StatusDemoApp::load(AppContext&) noexcept {
  ++stats_.load_count;
  elapsed_ms_ = 0;
  return true;
}

void StatusDemoApp::unload(AppContext&) noexcept {
  ++stats_.unload_count;
}

void StatusDemoApp::update(AppContext&,
                           const std::uint32_t elapsed_ms) noexcept {
  ++stats_.update_count;
  elapsed_ms_ += elapsed_ms;
}

void StatusDemoApp::render(AppContext& context) noexcept {
  ++stats_.render_count;
  context.canvas.clear(kStatusBackground);
  context.canvas.fillRect(24, 28, 72, 8, kStatusAccent);
  context.canvas.drawText(24, 62, "PLAYERTIA", kWhite);
  context.canvas.drawText(24, 98, "App shell ready", kStatusAccent);
  context.canvas.drawText(24, 352, "Top-right button", kMuted);
  context.canvas.drawText(24, 378, "Switch application", kWhite);

  const auto pulse_width =
      static_cast<std::int16_t>(24U + ((elapsed_ms_ / 20U) % 160U));
  context.canvas.fillRect(24, 424, pulse_width, 4, kStatusAccent);
}

const DemoLifecycleStats& StatusDemoApp::stats() const noexcept {
  return stats_;
}

std::string_view PlaygroundDemoApp::id() const noexcept {
  return "demo.playground";
}

std::string_view PlaygroundDemoApp::title() const noexcept {
  return "Playground";
}

bool PlaygroundDemoApp::load(AppContext&) noexcept {
  ++stats_.load_count;
  elapsed_ms_ = 0;
  return true;
}

void PlaygroundDemoApp::unload(AppContext&) noexcept {
  ++stats_.unload_count;
}

void PlaygroundDemoApp::update(AppContext&,
                               const std::uint32_t elapsed_ms) noexcept {
  ++stats_.update_count;
  elapsed_ms_ += elapsed_ms;
}

void PlaygroundDemoApp::render(AppContext& context) noexcept {
  ++stats_.render_count;
  context.canvas.clear(kPlaygroundBackground);
  context.canvas.drawText(24, 38, "PLACEHOLDER 02", kMuted);
  context.canvas.drawText(24, 72, "Tiny playground", kWhite);

  const auto offset =
      static_cast<std::int16_t>((elapsed_ms_ / 16U) % 44U);
  context.canvas.fillCircle(
      static_cast<std::int16_t>(110 + offset), 226, 34, kPlaygroundAccent);
  context.canvas.fillCircle(
      static_cast<std::int16_t>(258 - offset), 226, 22, kPlaygroundSecondary);

  context.canvas.drawText(24, 378, "No real app yet", kMuted);
  context.canvas.drawText(24, 404, "Press top-right to return", kWhite);
}

const DemoLifecycleStats& PlaygroundDemoApp::stats() const noexcept {
  return stats_;
}

}  // namespace playertia::esp32_s3
