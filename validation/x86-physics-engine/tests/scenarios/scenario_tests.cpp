#include "dial/test_support.hpp"

#include <gtest/gtest.h>

#include <array>

namespace dial {
namespace {

TEST(Scenarios, HorizontalStillHasNoPlanarGravityTorque) {
  PhysicalDialEngine engine;
  engine.reset({0.3, -0.7, 1.2}, {}, {});
  StepInput input = test::makeInput();
  for (TimeTarget& target : input.timeTargets) {
    target.valid = false;
  }
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(
      test::makeBase(0U, {0.0F, 0.0F, -9.80665F}), input));
  EXPECT_NEAR(engine.snapshot().hands[0].torque.effectiveFieldNm, 0.0F,
              1.0e-10F);
  EXPECT_NEAR(engine.snapshot().hands[1].torque.effectiveFieldNm, 0.0F,
              1.0e-10F);
  EXPECT_NEAR(engine.snapshot().hands[2].torque.effectiveFieldNm, 0.0F,
              1.0e-10F);
}

TEST(Scenarios, TouchOnlyConstrainsSelectedHand) {
  PhysicalDialEngine engine;
  engine.reset({}, {}, {});
  StepInput baseline = test::makeInput();
  for (TimeTarget& target : baseline.timeTargets) {
    target.valid = false;
  }
  baseline.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  baseline.eventCount = 1;
  ASSERT_TRUE(engine.step(
      test::makeBase(0U, {2.0F, 0.0F, 0.0F}), baseline));
  const auto before = engine.snapshot();

  StepInput touch = baseline;
  touch.eventCount = 0;
  touch.touchTargets[0] = {true, false, 1.0F, 0.0F, 1.0F, 10'000U};
  ASSERT_TRUE(engine.step(
      test::makeBase(10'000U, {2.0F, 0.0F, 0.0F}), touch));
  EXPECT_EQ(engine.snapshot().hands[0].mode, HandMode::TouchConstrained);
  EXPECT_NE(engine.snapshot().hands[1].angleUnwrappedRad,
            before.hands[1].angleUnwrappedRad);
  EXPECT_NE(engine.snapshot().hands[2].angleUnwrappedRad,
            before.hands[2].angleUnwrappedRad);
}

TEST(Scenarios, ReleasePreservesDeclaredVelocity) {
  PhysicalDialEngine engine;
  engine.reset({}, {}, {});
  StepInput input = test::makeInput();
  for (TimeTarget& target : input.timeTargets) {
    target.valid = false;
  }
  input.touchTargets[0] = {true, false, 0.5F, 2.0F, 1.0F, 0U};
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
  input.touchTargets[0] = {false, true, 0.5F, 3.0F, 1.0F, 10'000U};
  ASSERT_TRUE(engine.step(test::makeBase(10'000U, {}), input));
  EXPECT_GT(engine.snapshot().hands[0].angularVelocityRadSec, 2.0F);
  EXPECT_NE(engine.snapshot().hands[0].safetyFlags &
                SafetyTouchReleased,
            0U);
}

TEST(Scenarios, ReleaseMomentumIncludesMovingBaseOmega) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].viscousDampingNmSecPerRad = 0.0F;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {0.0F, 0.0F, 2.0F});
  StepInput input = test::makeInput();
  for (TimeTarget& target : input.timeTargets) {
    target.valid = false;
  }
  input.touchTargets[0] = {false, true, 0.0F, 3.0F, 1.0F, 0U};
  ASSERT_TRUE(engine.step(
      test::makeBase(0U, {}, {0.0F, 0.0F, 2.0F},
                     {0.0F, 0.0F, 2.0F}),
      input));
  EXPECT_NEAR(engine.snapshot().hands[0].angularVelocityRadSec, 3.0F,
              0.05F);
  EXPECT_NEAR(engine.snapshot().hands[0].reducedMomentumKgM2RadSec,
              engine.config().hands[0].inertiaPivotKgM2 * 5.0F,
              1.0e-11F);
}

TEST(Scenarios, TimeReferenceNeverInstantlyOverwritesMechanicalAngle) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.returnStrategy = ReturnStrategy::Mechanical;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {});
  StepInput input = test::makeInput(2.5F);
  input.events[0] = {EventType::BeginReturn, 0, 0.0F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
  EXPECT_LT(std::abs(engine.snapshot().hands[0].angleUnwrappedRad), 0.5);
  EXPECT_GT(std::abs(engine.snapshot().hands[0].torque.timeNm), 0.0F);
}

TEST(Scenarios, AnimationSynchronizesVelocityAuthorityOnCompletion) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.returnStrategy = ReturnStrategy::Animation;
  config.animationDurationSec = 0.05F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({-1.0, -1.0, -1.0}, {}, {});
  StepInput input = test::makeInput(0.7F, 0.1F);
  input.events[0] = {EventType::BeginReturn, 0, 0.0F, 0U};
  input.eventCount = 1;
  for (std::uint64_t step = 0; step < 8U; ++step) {
    input.eventCount = step == 0U ? 1U : 0U;
    ASSERT_TRUE(engine.step(test::makeBase(step * 10'000U, {}), input));
  }
  EXPECT_NEAR(shortestAngle(
                  static_cast<float>(
                      engine.snapshot().hands[0].angleUnwrappedRad),
                  0.7F),
              0.0F, 0.03F);
  EXPECT_TRUE(engine.snapshot().hands[0].stateValid);
}

TEST(Scenarios, SameInputIsBitDeterministic) {
  PhysicalDialEngine first;
  PhysicalDialEngine second;
  const std::array<double, kHandCount> angles{0.1, -0.2, 0.3};
  const std::array<float, kHandCount> velocity{1.0F, -2.0F, 0.5F};
  first.reset(angles, velocity, {});
  second.reset(angles, velocity, {});
  StepInput input = test::makeInput();
  for (std::uint64_t step = 0; step < 1000U; ++step) {
    const float phase = static_cast<float>(step) * 0.01F;
    const Vec3f acceleration{std::sin(phase) * 3.0F,
                             std::cos(phase * 0.7F) * 2.0F, 0.0F};
    const Vec3f omega{0.3F * std::sin(phase), 0.2F * std::cos(phase),
                      0.1F};
    const BaseInterval base =
        test::makeBase(step * 10'000U, acceleration, omega, omega);
    ASSERT_TRUE(first.step(base, input));
    ASSERT_TRUE(second.step(base, input));
  }
  EXPECT_EQ(deterministicHash(first.snapshot()),
            deterministicHash(second.snapshot()));
}

}  // namespace
}  // namespace dial
