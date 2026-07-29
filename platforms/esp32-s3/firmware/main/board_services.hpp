#pragma once

#include <cstdint>

#include "dial/physics.hpp"
#include "pcf85063a.h"
#include "playertia/clock/clock_model.hpp"
#include "qmi8658.h"

namespace playertia::esp32_s3 {

using UtcDateTime = playertia::clock::DateTime;

struct MotionFrame {
  dial::Vec3f specific_force_body_mps2{};
  dial::Vec3f angular_velocity_body_rad_s{};
  dial::Vec3f angular_acceleration_body_rad_s2{};
  dial::Quatf body_to_world{};
  dial::Vec3f gravity_body_mps2{};
  dial::Vec3f effective_acceleration_body_mps2{};
  std::uint64_t timestamp_us{};
  std::uint32_t quality_flags{};
  bool valid{};
};

class BoardServices {
 public:
  bool initialize(std::int64_t build_epoch_utc) noexcept;
  bool setUtcEpoch(std::int64_t epoch_utc) noexcept;
  [[nodiscard]] double utcEpochSeconds() noexcept;
  [[nodiscard]] UtcDateTime localDateTime(
      std::int32_t utc_offset_seconds) noexcept;
  [[nodiscard]] MotionFrame sampleMotion() noexcept;
  [[nodiscard]] bool rtcReady() const noexcept;
  [[nodiscard]] bool imuReady() const noexcept;

 private:
  bool initializeRtc(std::int64_t build_epoch_utc) noexcept;
  bool initializeImu() noexcept;
  bool readRtcEpoch(std::int64_t& epoch_utc) noexcept;
  void updateAttitude(dial::Vec3f specific_force,
                      dial::Vec3f angular_velocity,
                      float dt) noexcept;

  pcf85063a_dev_t rtc_{};
  qmi8658_dev_t imu_{};
  dial::Quatf body_to_world_{};
  dial::Vec3f gyro_integral_{};
  dial::Vec3f previous_omega_{};
  std::int64_t epoch_at_sync_{};
  std::uint64_t monotonic_at_sync_us_{};
  std::uint64_t previous_motion_us_{};
  std::uint64_t last_rtc_refresh_us_{};
  bool rtc_ready_{};
  bool imu_ready_{};
  bool attitude_initialized_{};
};

}  // namespace playertia::esp32_s3
