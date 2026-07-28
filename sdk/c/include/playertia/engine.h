#ifndef PLAYERTIA_ENGINE_H
#define PLAYERTIA_ENGINE_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(PLAYERTIA_SDK_BUILDING_LIBRARY)
#    define PLAYERTIA_API __declspec(dllexport)
#  else
#    define PLAYERTIA_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define PLAYERTIA_API __attribute__((visibility("default")))
#else
#  define PLAYERTIA_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PLAYERTIA_ABI_VERSION UINT32_C(0x00010000)
#define PLAYERTIA_HAND_COUNT 3U
#define PLAYERTIA_BASE_QUARTER_COUNT 4U
#define PLAYERTIA_MAX_EVENTS 16U
#define PLAYERTIA_MAX_CONTACTS 8U

typedef struct playertia_engine playertia_engine;

typedef enum playertia_result {
  PLAYERTIA_OK = 0,
  PLAYERTIA_INVALID_ARGUMENT = 1,
  PLAYERTIA_ALLOCATION_FAILED = 2,
  PLAYERTIA_ENGINE_REJECTED_INPUT = 3,
  PLAYERTIA_ABI_MISMATCH = 4
} playertia_result;

typedef enum playertia_event_type {
  PLAYERTIA_EVENT_ENTER_PHYSICS = 0,
  PLAYERTIA_EVENT_BEGIN_RETURN = 1,
  PLAYERTIA_EVENT_FORCE_TIME_TRACKING = 2,
  PLAYERTIA_EVENT_RECOVER = 3,
  PLAYERTIA_EVENT_DRIVE_IMPULSE = 4,
  PLAYERTIA_EVENT_EXTERNAL_IMPULSE = 5
} playertia_event_type;

typedef struct playertia_vec3 {
  float x;
  float y;
  float z;
} playertia_vec3;

typedef struct playertia_base_subinterval {
  uint64_t start_us;
  uint64_t end_us;
  playertia_vec3 effective_accel_impulse_mps;
  float omega_outer_product_integral[9];
  playertia_vec3 omega_body_at_start_rad_sec;
  playertia_vec3 omega_body_at_end_rad_sec;
  playertia_vec3 gravity_body_at_end_mps2;
  uint32_t quality_flags;
} playertia_base_subinterval;

typedef struct playertia_base_interval {
  uint32_t abi_version;
  uint32_t struct_size;
  uint64_t start_us;
  uint64_t end_us;
  playertia_base_subinterval quarter[PLAYERTIA_BASE_QUARTER_COUNT];
  uint32_t quality_flags;
} playertia_base_interval;

typedef struct playertia_time_target {
  float angle_unwrapped_rad;
  float angular_velocity_rad_sec;
  float angular_acceleration_rad_sec2;
  uint64_t timestamp_us;
  uint8_t valid;
} playertia_time_target;

typedef struct playertia_touch_target {
  float angle_unwrapped_rad;
  float angular_velocity_rad_sec;
  float confidence;
  uint64_t timestamp_us;
  uint8_t active;
  uint8_t release;
} playertia_touch_target;

typedef struct playertia_event {
  uint64_t timestamp_us;
  float scalar;
  uint8_t type;
  uint8_t hand;
} playertia_event;

typedef struct playertia_contact {
  float jacobian_a[2];
  float jacobian_b[2];
  float gap_m;
  float restitution;
  float bias_velocity_rad_sec;
  uint64_t event_id;
  int8_t hand_a;
  int8_t hand_b;
  uint8_t active;
} playertia_contact;

typedef struct playertia_step_input {
  uint32_t abi_version;
  uint32_t struct_size;
  playertia_time_target time_targets[PLAYERTIA_HAND_COUNT];
  playertia_touch_target touch_targets[PLAYERTIA_HAND_COUNT];
  float drive_torque_nm[PLAYERTIA_HAND_COUNT];
  playertia_event events[PLAYERTIA_MAX_EVENTS];
  uint32_t event_count;
  playertia_contact contacts[PLAYERTIA_MAX_CONTACTS];
  uint32_t contact_count;
  uint8_t intentional_interaction;
} playertia_step_input;

typedef struct playertia_hand_snapshot {
  double angle_unwrapped_rad;
  float angular_velocity_rad_sec;
  float reduced_momentum_kg_m2_rad_sec;
  float bend_angle_rad;
  float bend_velocity_rad_sec;
  playertia_vec3 axis_body;
  playertia_vec3 center_of_mass_body_m;
  playertia_vec3 tip_body_m;
  uint32_t safety_flags;
  uint8_t kind;
  uint8_t mode;
  uint8_t state_valid;
  uint8_t momentum_authoritative;
  uint8_t settled;
} playertia_hand_snapshot;

typedef struct playertia_snapshot {
  uint32_t abi_version;
  uint32_t struct_size;
  uint64_t sequence;
  uint64_t timestamp_us;
  uint64_t deterministic_hash;
  playertia_hand_snapshot hands[PLAYERTIA_HAND_COUNT];
  uint32_t rejected_intervals;
  uint32_t consumed_intervals;
  uint32_t degradation_count;
  uint16_t last_error;
  uint8_t global_mode;
} playertia_snapshot;

PLAYERTIA_API uint32_t playertia_engine_get_abi_version(void);
PLAYERTIA_API size_t playertia_engine_get_state_size(void);
PLAYERTIA_API playertia_result playertia_engine_create(uint32_t abi_version, playertia_engine** out_engine);
PLAYERTIA_API void playertia_engine_destroy(playertia_engine* engine);
PLAYERTIA_API playertia_result playertia_engine_reset(
    playertia_engine* engine,
    const double angles_rad[PLAYERTIA_HAND_COUNT],
    const float relative_velocities_rad_sec[PLAYERTIA_HAND_COUNT],
    playertia_vec3 base_omega_rad_sec);
PLAYERTIA_API playertia_result playertia_engine_step(
    playertia_engine* engine,
    const playertia_base_interval* interval,
    const playertia_step_input* input);
PLAYERTIA_API playertia_result playertia_engine_get_snapshot(
    const playertia_engine* engine,
    playertia_snapshot* out_snapshot,
    uint32_t out_snapshot_size);

#ifdef __cplusplus
}
#endif

#endif
