#include "board_services.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace playertia::esp32_s3 {
namespace {

constexpr char kTag[] = "board_services";
constexpr std::uint8_t kRtcAddress = PCF85063A_ADDRESS;
constexpr std::uint32_t kProbeTimeoutMs = 100;
constexpr float kGravity = 9.80665F;
constexpr float kDegreesToRadians = dial::kPi / 180.0F;
constexpr float kMahonyKp = 2.4F;
constexpr float kMahonyKi = 0.04F;
constexpr float kAccelGateMps2 = 2.5F;
constexpr std::uint64_t kRtcRefreshPeriodUs = 60'000'000ULL;
constexpr std::uint8_t kResetRegister = 0x60;
constexpr std::uint8_t kResetCommand = 0xB0;
constexpr std::uint8_t kCtrl1Value = 0x60;

constexpr dial::Vec3f sensorToDial(const dial::Vec3f value) noexcept {
  // The V2 board mounts QMI8658 X toward the display's 12 o'clock edge,
  // while the dial simulation uses X toward 3 o'clock and Y toward
  // 6 o'clock. Rotate every IMU vector by -90 degrees around screen Z.
  return {value.y, -value.x, value.z};
}

static_assert(sensorToDial({-1.0F, 0.0F, 0.0F}).x == 0.0F);
static_assert(sensorToDial({-1.0F, 0.0F, 0.0F}).y == 1.0F);
static_assert(sensorToDial({0.0F, 0.0F, 1.0F}).z == 1.0F);

dial::Quatf multiply(const dial::Quatf a, const dial::Quatf b) noexcept {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

bool reasonable(const pcf85063a_datetime_t& value) noexcept {
  return value.year >= 2024U && value.year <= 2069U &&
         value.month >= 1U && value.month <= 12U &&
         value.day >= 1U && value.day <= 31U && value.dotw <= 6U &&
         value.hour <= 23U && value.min <= 59U && value.sec <= 59U;
}

}  // namespace

bool BoardServices::initialize(const std::int64_t build_epoch_utc) noexcept {
  const bool rtc_ok = initializeRtc(build_epoch_utc);
  const bool imu_ok = initializeImu();
  ESP_LOGI(kTag, "Board services: RTC=%s IMU=%s",
           rtc_ok ? "ready" : "unavailable",
           imu_ok ? "ready" : "unavailable");
  return rtc_ok || imu_ok;
}

bool BoardServices::initializeRtc(
    const std::int64_t build_epoch_utc) noexcept {
  i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
  if (bus == nullptr ||
      i2c_master_probe(bus, kRtcAddress, kProbeTimeoutMs) != ESP_OK ||
      pcf85063a_init(&rtc_, bus, kRtcAddress) != ESP_OK) {
    ESP_LOGE(kTag, "PCF85063A initialization failed");
    return false;
  }

  rtc_ready_ = true;
  std::int64_t current_epoch{};
  if (!readRtcEpoch(current_epoch) ||
      current_epoch + 30LL < build_epoch_utc) {
    ESP_LOGW(kTag, "RTC is unset or behind build time; seeding UTC");
    if (!setUtcEpoch(build_epoch_utc)) {
      rtc_ready_ = false;
      return false;
    }
  } else {
    epoch_at_sync_ = current_epoch;
    monotonic_at_sync_us_ = esp_timer_get_time();
  }
  last_rtc_refresh_us_ = esp_timer_get_time();
  ESP_LOGI(kTag, "RTC UTC epoch=%lld",
           static_cast<long long>(epoch_at_sync_));
  return true;
}

bool BoardServices::initializeImu() noexcept {
  i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
  if (bus == nullptr) {
    return false;
  }
  std::uint8_t address{};
  constexpr std::uint8_t candidates[]{
      QMI8658_ADDRESS_HIGH, QMI8658_ADDRESS_LOW};
  for (const std::uint8_t candidate : candidates) {
    if (i2c_master_probe(bus, candidate, kProbeTimeoutMs) == ESP_OK) {
      address = candidate;
      break;
    }
  }
  if (address == 0U || qmi8658_init(&imu_, bus, address) != ESP_OK) {
    ESP_LOGE(kTag, "QMI8658 not found");
    return false;
  }
  if (qmi8658_write_register(&imu_, kResetRegister, kResetCommand) !=
      ESP_OK) {
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(20));
  if (qmi8658_write_register(&imu_, QMI8658_CTRL1, kCtrl1Value) != ESP_OK ||
      qmi8658_set_accel_range(&imu_, QMI8658_ACCEL_RANGE_16G) != ESP_OK ||
      qmi8658_set_accel_odr(&imu_, QMI8658_ACCEL_ODR_250HZ) != ESP_OK ||
      qmi8658_set_gyro_range(&imu_, QMI8658_GYRO_RANGE_2048DPS) != ESP_OK ||
      qmi8658_set_gyro_odr(&imu_, QMI8658_GYRO_ODR_250HZ) != ESP_OK) {
    ESP_LOGE(kTag, "QMI8658 configuration failed");
    return false;
  }
  qmi8658_set_accel_unit_mps2(&imu_, true);
  qmi8658_set_gyro_unit_dps(&imu_, true);
  if (qmi8658_enable_sensors(
          &imu_, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO) != ESP_OK) {
    return false;
  }
  imu_ready_ = true;
  ESP_LOGI(kTag, "QMI8658 ready at 0x%02x, 16g / 2048dps / 250Hz",
           address);
  return true;
}

bool BoardServices::readRtcEpoch(std::int64_t& epoch_utc) noexcept {
  pcf85063a_datetime_t value{};
  if (!rtc_ready_ ||
      pcf85063a_get_time_date(&rtc_, &value) != ESP_OK ||
      !reasonable(value)) {
    return false;
  }
  epoch_utc = playertia::clock::dateTimeToEpoch(
      {value.year, value.month, value.day, value.dotw,
       value.hour, value.min, value.sec});
  return true;
}

bool BoardServices::setUtcEpoch(const std::int64_t epoch_utc) noexcept {
  if (!rtc_ready_) {
    return false;
  }
  const UtcDateTime value =
      playertia::clock::epochToDateTime(epoch_utc);
  pcf85063a_datetime_t rtc_value{};
  rtc_value.year = value.year;
  rtc_value.month = value.month;
  rtc_value.day = value.day;
  rtc_value.dotw = value.weekday;
  rtc_value.hour = value.hour;
  rtc_value.min = value.minute;
  rtc_value.sec = value.second;
  if (pcf85063a_set_time_date(&rtc_, rtc_value) != ESP_OK) {
    return false;
  }
  epoch_at_sync_ = epoch_utc;
  monotonic_at_sync_us_ = esp_timer_get_time();
  last_rtc_refresh_us_ = monotonic_at_sync_us_;
  return true;
}

double BoardServices::utcEpochSeconds() noexcept {
  const std::uint64_t now = esp_timer_get_time();
  if (rtc_ready_ && now - last_rtc_refresh_us_ >= kRtcRefreshPeriodUs) {
    std::int64_t refreshed{};
    if (readRtcEpoch(refreshed)) {
      epoch_at_sync_ = refreshed;
      monotonic_at_sync_us_ = now;
      last_rtc_refresh_us_ = now;
    }
  }
  return static_cast<double>(epoch_at_sync_) +
         static_cast<double>(now - monotonic_at_sync_us_) * 1.0e-6;
}

UtcDateTime BoardServices::localDateTime(
    const std::int32_t utc_offset_seconds) noexcept {
  return playertia::clock::epochToDateTime(
      static_cast<std::int64_t>(utcEpochSeconds()) +
      static_cast<std::int64_t>(utc_offset_seconds));
}

void BoardServices::updateAttitude(
    const dial::Vec3f specific_force,
    dial::Vec3f angular_velocity,
    const float dt) noexcept {
  const float acceleration_norm = dial::norm(specific_force);
  if (!attitude_initialized_ && acceleration_norm > 1.0F) {
    const dial::Vec3f measured_up =
        specific_force * (1.0F / acceleration_norm);
    const float roll = std::atan2(measured_up.y, measured_up.z);
    const float pitch =
        std::atan2(-measured_up.x,
                   std::sqrt(measured_up.y * measured_up.y +
                             measured_up.z * measured_up.z));
    const float cr = std::cos(roll * 0.5F);
    const float sr = std::sin(roll * 0.5F);
    const float cp = std::cos(pitch * 0.5F);
    const float sp = std::sin(pitch * 0.5F);
    body_to_world_ = dial::normalized(
        {cr * cp, sr * cp, cr * sp, -sr * sp});
    attitude_initialized_ = true;
  }

  if (std::abs(acceleration_norm - kGravity) < kAccelGateMps2 &&
      acceleration_norm > 1.0F) {
    const dial::Vec3f measured_up =
        specific_force * (1.0F / acceleration_norm);
    const dial::Vec3f predicted_up =
        dial::rotateWorldToBody(body_to_world_, {0.0F, 0.0F, 1.0F});
    const dial::Vec3f error = dial::cross(measured_up, predicted_up);
    gyro_integral_ =
        gyro_integral_ + error * (kMahonyKi * dt);
    angular_velocity =
        angular_velocity + error * kMahonyKp + gyro_integral_;
  }

  const dial::Quatf omega{
      0.0F, angular_velocity.x, angular_velocity.y, angular_velocity.z};
  const dial::Quatf derivative = multiply(body_to_world_, omega);
  body_to_world_.w += 0.5F * derivative.w * dt;
  body_to_world_.x += 0.5F * derivative.x * dt;
  body_to_world_.y += 0.5F * derivative.y * dt;
  body_to_world_.z += 0.5F * derivative.z * dt;
  body_to_world_ = dial::normalized(body_to_world_);
}

MotionFrame BoardServices::sampleMotion() noexcept {
  MotionFrame frame{};
  frame.timestamp_us = esp_timer_get_time();
  if (!imu_ready_) {
    frame.gravity_body_mps2 =
        dial::gravityBody(body_to_world_);
    return frame;
  }

  bool ready{};
  if (qmi8658_is_data_ready(&imu_, &ready) != ESP_OK || !ready) {
    return frame;
  }
  qmi8658_data_t raw{};
  if (qmi8658_read_sensor_data(&imu_, &raw) != ESP_OK) {
    return frame;
  }

  frame.specific_force_body_mps2 =
      {raw.accelX, raw.accelY, raw.accelZ};
  frame.angular_velocity_body_rad_s =
      {raw.gyroX * kDegreesToRadians,
       raw.gyroY * kDegreesToRadians,
       raw.gyroZ * kDegreesToRadians};
  const float dt =
      previous_motion_us_ == 0U
          ? 0.01F
          : std::clamp(
                static_cast<float>(frame.timestamp_us - previous_motion_us_) *
                    1.0e-6F,
                0.001F, 0.05F);
  frame.angular_acceleration_body_rad_s2 =
      (frame.angular_velocity_body_rad_s - previous_omega_) * (1.0F / dt);
  previous_omega_ = frame.angular_velocity_body_rad_s;
  previous_motion_us_ = frame.timestamp_us;
  updateAttitude(frame.specific_force_body_mps2,
                 frame.angular_velocity_body_rad_s, dt);
  frame.body_to_world = body_to_world_;
  frame.gravity_body_mps2 = dial::gravityBody(body_to_world_);
  frame.effective_acceleration_body_mps2 =
      dial::effectiveAccelerationAtCenter(
          frame.specific_force_body_mps2,
          frame.angular_velocity_body_rad_s,
          frame.angular_acceleration_body_rad_s2, {});
  if (dial::norm(frame.specific_force_body_mps2) > 150.0F ||
      dial::norm(frame.angular_velocity_body_rad_s) > 34.0F) {
    frame.quality_flags |= dial::QualitySaturated;
  }

  frame.specific_force_body_mps2 =
      sensorToDial(frame.specific_force_body_mps2);
  frame.angular_velocity_body_rad_s =
      sensorToDial(frame.angular_velocity_body_rad_s);
  frame.angular_acceleration_body_rad_s2 =
      sensorToDial(frame.angular_acceleration_body_rad_s2);
  frame.gravity_body_mps2 =
      sensorToDial(frame.gravity_body_mps2);
  frame.effective_acceleration_body_mps2 =
      sensorToDial(frame.effective_acceleration_body_mps2);
  constexpr float kSqrtHalf = 0.7071067811865476F;
  frame.body_to_world =
      multiply(body_to_world_, {kSqrtHalf, 0.0F, 0.0F, kSqrtHalf});
  frame.valid = true;
  return frame;
}

bool BoardServices::rtcReady() const noexcept { return rtc_ready_; }
bool BoardServices::imuReady() const noexcept { return imu_ready_; }

}  // namespace playertia::esp32_s3
