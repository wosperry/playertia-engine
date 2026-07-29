#pragma once

// Portable production physics core. Platform adapters and validation code
// live outside this library.

#include <array>
#include <cstddef>
#include <cstdint>

namespace dial {

constexpr std::size_t kHandCount = 3;
constexpr std::size_t kBaseQuarterCount = 4;
constexpr std::size_t kMaxContacts = 8;
constexpr std::size_t kMaxEvents = 16;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

struct Vec3f {
  float x{};
  float y{};
  float z{};
};

struct Quatf {
  float w{1.0F};
  float x{};
  float y{};
  float z{};
};

struct Mat3f {
  float v[3][3]{};
};

Vec3f operator+(Vec3f a, Vec3f b) noexcept;
Vec3f operator-(Vec3f a, Vec3f b) noexcept;
Vec3f operator*(Vec3f a, float s) noexcept;
float dot(Vec3f a, Vec3f b) noexcept;
Vec3f cross(Vec3f a, Vec3f b) noexcept;
float norm(Vec3f a) noexcept;
bool finite(Vec3f a) noexcept;
Quatf normalized(Quatf q, bool* corrected = nullptr) noexcept;
Vec3f rotateBodyToWorld(Quatf qWB, Vec3f body) noexcept;
Vec3f rotateWorldToBody(Quatf qWB, Vec3f world) noexcept;
Vec3f gravityBody(Quatf qWB, float gravity = 9.80665F) noexcept;
Vec3f effectiveAccelerationAtCenter(
    Vec3f specificForceAtSensor, Vec3f bodyAngularVelocity,
    Vec3f bodyAngularAcceleration, Vec3f centerToSensorBody) noexcept;
Vec3f handAxis(float angle) noexcept;
Vec3f handTangent(float angle) noexcept;
float shortestAngle(float from, float to) noexcept;

struct LogicalAngle {
  std::int64_t turns{};
  float phase{};

  [[nodiscard]] double value() const noexcept;
  void set(double radians) noexcept;
  void advance(float deltaRadians) noexcept;
};

enum class HandKind : std::uint8_t { Rigid, Flexible2D };
enum class HandMode : std::uint8_t {
  TimeControlled,
  ImuFree,
  TouchConstrained,
  Returning
};
enum class GlobalMode : std::uint8_t {
  TimeTracking,
  PhysicsPlay,
  AutoReturn,
  Degraded
};
enum class ReturnStrategy : std::uint8_t { Mechanical, Animation, Hybrid };
enum class EventType : std::uint8_t {
  EnterPhysics,
  BeginReturn,
  ForceTimeTracking,
  Recover,
  DriveImpulse,
  ExternalImpulse
};
enum class ErrorCode : std::uint16_t {
  None,
  InvalidConfiguration,
  InvalidInterval,
  NonFiniteInput,
  TimestampDiscontinuity,
  SubstepPolicyExceeded,
  ContactRejected
};

enum QualityFlags : std::uint32_t {
  QualityValid = 0,
  QualityGap = 1U << 0U,
  QualityDuplicate = 1U << 1U,
  QualityOutOfOrder = 1U << 2U,
  QualitySaturated = 1U << 3U,
  QualityDegraded = 1U << 4U
};

enum SafetyFlags : std::uint32_t {
  SafetyNone = 0,
  SafetySpeedLimited = 1U << 0U,
  SafetyEnergyLimited = 1U << 1U,
  SafetyRecovered = 1U << 2U,
  SafetyTouchReleased = 1U << 3U,
  SafetyContactApplied = 1U << 4U
};

struct FlexibleDefinition {
  float m1Kg{};
  float m2Kg{};
  float d1M{};
  float l1M{};
  float l2M{};
  float j1KgM2{};
  float j2KgM2{};
  float bendStiffnessNmRad{};
  float bendDampingNmSecRad{};
  float releaseAngleRad{0.35F};
  float releaseEnergyJ{1.0e-7F};
};

struct HandDefinition {
  HandKind kind{HandKind::Rigid};
  float massKg{};
  float visualLengthM{};
  Vec3f centerOfMassAtZeroM{};
  float inertiaPivotKgM2{};

  float viscousDampingNmSecPerRad{};
  float quadraticDampingNmSec2PerRad2{};
  float coulombFrictionNm{};
  float frictionSmoothVelocityRadSec{0.01F};

  float timeKpNmPerRad{};
  float timeKdNmSecPerRad{};
  float maxTimeTorqueNm{};
  float touchKpNmPerRad{};
  float touchKdNmSecPerRad{};
  float maxTouchTorqueNm{};

  float maxAngularSpeedRadSec{80.0F};
  float maxRelativeKineticEnergyJ{1.0e-4F};
  FlexibleDefinition flex{};
};

struct EngineConfig {
  std::array<HandDefinition, kHandCount> hands{};
  ReturnStrategy returnStrategy{ReturnStrategy::Hybrid};
  float baseDtSec{0.01F};
  std::uint8_t substeps{4};
  float idleTimeoutSec{5.0F};
  float settleAngleRad{0.01F};
  float settleVelocityRadSec{0.05F};
  float settleDurationSec{0.25F};
  float hybridCaptureAngleRad{0.08F};
  float animationDurationSec{0.25F};
  bool animationPreserveEntryVelocity{true};
  float maxEffectiveAccelerationMps2{156.9064F};
  float maxBodyAngularVelocityRadSec{35.744F};
};

EngineConfig makeDefaultConfig() noexcept;
float maximumDimensionlessSubstep(const EngineConfig& config) noexcept;

struct BaseSubInterval {
  std::uint64_t startUs{};
  std::uint64_t endUs{};
  Vec3f effectiveAccelImpulseMps{};
  Mat3f omegaOuterProductIntegral{};
  Vec3f omegaBodyAtStartRadSec{};
  Vec3f omegaBodyAtEndRadSec{};
  Vec3f gravityBodyAtEndMps2{};
  std::uint32_t qualityFlags{};
};

struct BaseInterval {
  std::uint64_t startUs{};
  std::uint64_t endUs{};
  std::array<BaseSubInterval, kBaseQuarterCount> quarter{};
  std::uint32_t qualityFlags{};
};

struct TimeTarget {
  float angleUnwrappedRad{};
  float angularVelocityRadSec{};
  float angularAccelerationRadSec2{};
  std::uint64_t timestampUs{};
  bool valid{true};
};

struct TouchTarget {
  bool active{};
  bool release{};
  float angleUnwrappedRad{};
  float angularVelocityRadSec{};
  float confidence{1.0F};
  std::uint64_t timestampUs{};
};

struct PhysicsEvent {
  EventType type{EventType::EnterPhysics};
  std::uint8_t hand{};
  float scalar{};
  std::uint64_t timestampUs{};
};

struct ContactConstraint {
  std::int8_t handA{-1};
  std::int8_t handB{-1};
  std::array<float, 2> jacobianA{};
  std::array<float, 2> jacobianB{};
  float gapM{};
  float restitution{};
  float biasVelocityRadSec{};
  std::uint64_t eventId{};
  bool active{};
};

struct StepInput {
  std::array<TimeTarget, kHandCount> timeTargets{};
  std::array<TouchTarget, kHandCount> touchTargets{};
  std::array<float, kHandCount> driveTorqueNm{};
  std::array<PhysicsEvent, kMaxEvents> events{};
  std::size_t eventCount{};
  std::array<ContactConstraint, kMaxContacts> contacts{};
  std::size_t contactCount{};
  bool intentionalInteraction{};
};

struct TorqueLedger {
  float effectiveFieldNm{};
  float rotatingBaseNm{};
  float driveNm{};
  float timeNm{};
  float touchNm{};
  float externalNm{};
  float elasticNm{};
  float collisionNm{};
  float dampingNm{};
  float totalNm{};
};

struct MomentumLedger {
  float effectiveField{};
  float rotatingBase{};
  float drive{};
  float time{};
  float touch{};
  float external{};
  float elastic{};
  float collision{};
  float damping{};
  float event{};
  float safety{};
  float residual{};
};

struct EnergyLedger {
  float kineticJ{};
  float gravityPotentialJ{};
  float timePotentialJ{};
  float flexPotentialJ{};
  float externalWorkJ{};
  float controlWorkJ{};
  float touchWorkJ{};
  float dampingDissipationJ{};
  float collisionEnergyDeltaJ{};
};

struct HandSnapshot {
  HandKind kind{HandKind::Rigid};
  HandMode mode{HandMode::TimeControlled};
  bool stateValid{};
  bool momentumAuthoritative{};
  bool settled{};
  double angleUnwrappedRad{};
  float angularVelocityRadSec{};
  float reducedMomentumKgM2RadSec{};
  float bendAngleRad{};
  float bendVelocityRadSec{};
  Vec3f axisBody{};
  Vec3f tangentBody{};
  Vec3f centerOfMassBodyM{};
  Vec3f tipBodyM{};
  TorqueLedger torque{};
  MomentumLedger momentum{};
  EnergyLedger energy{};
  std::uint32_t safetyFlags{};
};

struct EngineSnapshot {
  std::uint64_t sequence{};
  std::uint64_t timestampUs{};
  GlobalMode globalMode{GlobalMode::TimeTracking};
  std::array<HandSnapshot, kHandCount> hands{};
  ErrorCode lastError{ErrorCode::None};
  std::uint32_t rejectedIntervals{};
  std::uint32_t consumedIntervals{};
  std::uint32_t degradationCount{};
};

std::uint64_t deterministicHash(const EngineSnapshot& snapshot) noexcept;

class PhysicalDialEngine {
 public:
  PhysicalDialEngine() noexcept;

  bool configure(const EngineConfig& config) noexcept;
  void reset(const std::array<double, kHandCount>& angles,
             const std::array<float, kHandCount>& relativeVelocities,
             Vec3f baseOmega) noexcept;
  bool resetFlexibleState(std::size_t hand, double rootAngle, float bendAngle,
                          float rootVelocity,
                          float bendVelocity) noexcept;
  bool step(const BaseInterval& base, const StepInput& input) noexcept;
  [[nodiscard]] const EngineSnapshot& snapshot() const noexcept;
  [[nodiscard]] const EngineConfig& config() const noexcept;
  [[nodiscard]] std::size_t coreStateBytes() const noexcept;

 private:
  struct RigidState {
    LogicalAngle angle{};
    float momentum{};
  };
  struct FlexibleState {
    LogicalAngle rootAngle{};
    float beta{};
    float qDot[2]{};
  };
  struct AnimationState {
    bool active{};
    double startAngle{};
    float startVelocity{};
    std::uint64_t startUs{};
    std::uint64_t durationUs{};
  };
  struct RuntimeHand {
    RigidState rigid{};
    FlexibleState flexible{};
    AnimationState animation{};
    HandMode mode{HandMode::TimeControlled};
    bool settled{};
    float settledTimeSec{};
    std::uint32_t flags{};
    EnergyLedger energy{};
  };

  EngineConfig config_{};
  std::array<RuntimeHand, kHandCount> states_{};
  EngineSnapshot snapshot_{};
  GlobalMode globalMode_{GlobalMode::TimeTracking};
  bool configured_{};
  float idleTimeSec_{};
  std::uint64_t expectedStartUs_{};

  bool validateConfig(const EngineConfig& config) noexcept;
  bool validateInterval(const BaseInterval& base) noexcept;
  void applyEvents(const StepInput& input, Vec3f baseOmega) noexcept;
  void updateStateMachine(const StepInput& input, Vec3f baseOmega) noexcept;
  void integrateRigid(std::size_t index, const BaseSubInterval& interval,
                      const TimeTarget& time, const TouchTarget& touch,
                      float driveTorque, float dt) noexcept;
  void integrateFlexible(std::size_t index, const BaseSubInterval& interval,
                         const TimeTarget& time, const TouchTarget& touch,
                         float driveTorque, float dt) noexcept;
  void solveContacts(const StepInput& input, Vec3f omegaEnd) noexcept;
  void applyAnimation(std::size_t index, const TimeTarget& target,
                      Vec3f omegaEnd, std::uint64_t nowUs) noexcept;
  void publish(const BaseInterval& base) noexcept;
};

}  // namespace dial
