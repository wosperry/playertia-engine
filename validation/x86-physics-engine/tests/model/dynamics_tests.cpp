#include "dial/reference.hpp"
#include "dial/test_support.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace dial {
namespace {

TEST(RigidDynamics, ConstantDriveMatchesDiscreteMomentum) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.returnStrategy = ReturnStrategy::Mechanical;
  config.hands[0].viscousDampingNmSecPerRad = 0.0F;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {});
  StepInput input = test::makeInput();
  input.timeTargets[0].valid = false;
  input.driveTorqueNm[0] = 1.0e-8F;
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
  const float expected = input.driveTorqueNm[0] * config.baseDtSec;
  EXPECT_NEAR(engine.snapshot().hands[0].reducedMomentumKgM2RadSec,
              expected, 1.0e-12F);
  EXPECT_NEAR(engine.snapshot().hands[0].momentum.residual, 0.0F,
              1.0e-12F);
  const float expectedAngle =
      input.driveTorqueNm[0] /
      config.hands[0].inertiaPivotKgM2 *
      (0.0025F * 0.0025F) * 10.0F;
  EXPECT_NEAR(engine.snapshot().hands[0].angleUnwrappedRad,
              expectedAngle, 2.0e-6F);
}

TEST(RigidDynamics, EffectiveFieldTorqueHasCorrectSign) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].viscousDampingNmSecPerRad = 0.0F;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {});
  StepInput input = test::makeInput();
  input.timeTargets[0].valid = false;
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {1.0F, 0.0F, 0.0F}),
                          input));
  const float firstMoment =
      config.hands[0].massKg *
      -config.hands[0].centerOfMassAtZeroM.y;
  EXPECT_NEAR(engine.snapshot().hands[0].momentum.effectiveField,
              firstMoment * 0.01F, 2.0e-11F);
  EXPECT_GT(engine.snapshot().hands[0].angularVelocityRadSec, 0.0F);
}

TEST(RigidDynamics, ReducedMomentumDoesNotDoubleEulerTerm) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].viscousDampingNmSecPerRad = 0.0F;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {});
  StepInput input = test::makeInput();
  input.timeTargets[0].valid = false;
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(
      test::makeBase(0U, {}, {}, {0.0F, 0.0F, 1.0F}), input));
  EXPECT_NEAR(engine.snapshot().hands[0].reducedMomentumKgM2RadSec,
              0.0F, 1.0e-12F);
  EXPECT_NEAR(engine.snapshot().hands[0].angularVelocityRadSec, -1.0F,
              1.0e-5F);
}

TEST(RigidDynamics, EventImpulseConsumedExactlyOnce) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].viscousDampingNmSecPerRad = 0.0F;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {});
  StepInput input = test::makeInput();
  input.timeTargets[0].valid = false;
  input.events[0] = {EventType::ExternalImpulse, 0, 1.0e-8F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
  EXPECT_NEAR(engine.snapshot().hands[0].reducedMomentumKgM2RadSec,
              1.0e-8F, 1.0e-12F);
}

TEST(RigidDynamics, ConvergesTowardIndependentRk4) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.substeps = 4;
  config.returnStrategy = ReturnStrategy::Mechanical;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({0.35, 0.0, 0.0}, {}, {});
  StepInput input = test::makeInput();
  input.timeTargets[0].valid = false;
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;

  reference::RigidReferenceParameters parameters{
      config.hands[0].inertiaPivotKgM2,
      config.hands[0].massKg *
          -config.hands[0].centerOfMassAtZeroM.y,
      config.hands[0].viscousDampingNmSecPerRad,
      0.0L, 0.0L,
      config.hands[0].frictionSmoothVelocityRadSec};
  reference::RigidReferenceInput referenceInput{
      0.0L, 1.0L, 0.0L, 0.0L, 0.0L, 0.0L};
  reference::RigidReferenceState truth{0.35L, 0.0L};

  for (std::uint64_t step = 0; step < 100U; ++step) {
    input.eventCount = step == 0U ? 1U : 0U;
    ASSERT_TRUE(engine.step(
        test::makeBase(step * 10'000U, {0.0F, 1.0F, 0.0F}),
        input));
    truth = reference::integrateRigidRk4(
        truth, parameters, referenceInput, 0.01L, 2.5e-5L);
  }
  EXPECT_NEAR(engine.snapshot().hands[0].angleUnwrappedRad,
              static_cast<double>(truth.angle), 0.03);
  EXPECT_NEAR(engine.snapshot().hands[0].angularVelocityRadSec,
              static_cast<double>(truth.angularVelocity), 0.15);
}

TEST(FlexibleDynamics, MassMatrixIsSymmetricPositiveDefinite) {
  const FlexibleDefinition flex = makeDefaultConfig().hands[1].flex;
  for (int sample = 0; sample <= 1000; ++sample) {
    const long double beta =
        -static_cast<long double>(kPi) +
        2.0L * static_cast<long double>(kPi) * sample / 1000.0L;
    const auto mass = reference::flexibleMassMatrix(beta, flex);
    EXPECT_GT(mass.m11, 0.0L);
    EXPECT_GT(mass.m22, 0.0L);
    EXPECT_GT(mass.determinant(), 0.0L);
  }
}

TEST(FlexibleDynamics, InternalBendRestoresTowardZero) {
  const FlexibleDefinition flex = makeDefaultConfig().hands[1].flex;
  const auto acceleration = reference::solveFlexibleAcceleration(
      0.0L, 0.2L, 0.0L, 0.0L, flex, 0.0L, 0.0L);
  EXPECT_LT(acceleration[1], 0.0L);
}

TEST(FlexibleDynamics, InternalBendPreservesCyclicMomentum) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[1].viscousDampingNmSecPerRad = 0.0F;
  config.hands[1].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[1].coulombFrictionNm = 0.0F;
  config.hands[1].flex.bendDampingNmSecRad = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {}, {});
  ASSERT_TRUE(engine.resetFlexibleState(1, 0.0, 0.2F, 0.0F, 0.0F));
  StepInput input = test::makeInput();
  for (TimeTarget& target : input.timeTargets) {
    target.valid = false;
  }
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
  const auto mass = reference::flexibleMassMatrix(
      engine.snapshot().hands[1].bendAngleRad, config.hands[1].flex);
  const long double cyclicMomentum =
      mass.m11 * engine.snapshot().hands[1].angularVelocityRadSec +
      mass.m12 * engine.snapshot().hands[1].bendVelocityRadSec;
  EXPECT_NEAR(static_cast<double>(cyclicMomentum), 0.0, 1.0e-11);
}

TEST(Collision, ImpulseMomentumAndDuplicateConsumption) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].viscousDampingNmSecPerRad = 0.0F;
  config.hands[0].quadraticDampingNmSec2PerRad2 = 0.0F;
  config.hands[0].coulombFrictionNm = 0.0F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {-1.0F, 0.0F, 0.0F}, {});
  StepInput input = test::makeInput();
  input.timeTargets[0].valid = false;
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  input.contacts[0] = {0, -1, {1.0F, 0.0F}, {}, -0.001F, 1.0F,
                       0.0F, 42U, true};
  input.contacts[1] = input.contacts[0];
  input.contactCount = 2;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
  EXPECT_NEAR(engine.snapshot().hands[0].angularVelocityRadSec, 1.0F,
              0.02F);
  EXPECT_EQ(engine.snapshot().lastError, ErrorCode::ContactRejected);
}

}  // namespace
}  // namespace dial
