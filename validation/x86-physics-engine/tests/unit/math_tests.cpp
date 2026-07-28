#include "dial/physics.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace dial {
namespace {

TEST(Coordinates, QuaternionIdentityAndRoundTrip) {
  const Vec3f input{1.25F, -2.0F, 0.75F};
  const Quatf quarterTurn{std::cos(kPi / 4.0F), 0.0F, 0.0F,
                          std::sin(kPi / 4.0F)};
  const Vec3f world = rotateBodyToWorld(quarterTurn, input);
  const Vec3f roundTrip = rotateWorldToBody(quarterTurn, world);
  EXPECT_NEAR(roundTrip.x, input.x, 3.0e-5F);
  EXPECT_NEAR(roundTrip.y, input.y, 3.0e-5F);
  EXPECT_NEAR(roundTrip.z, input.z, 3.0e-5F);
}

TEST(Coordinates, GravityMagnitudeAndQuaternionSign) {
  const Quatf q{0.70710678F, 0.70710678F, 0.0F, 0.0F};
  const Vec3f positive = gravityBody(q);
  const Vec3f negative = gravityBody({-q.w, -q.x, -q.y, -q.z});
  EXPECT_NEAR(norm(positive), 9.80665F, 3.0e-4F);
  EXPECT_NEAR(positive.x, negative.x, 1.0e-6F);
  EXPECT_NEAR(positive.y, negative.y, 1.0e-6F);
  EXPECT_NEAR(positive.z, negative.z, 1.0e-6F);
}

TEST(Coordinates, ClockGeometryConvention) {
  const std::array<Vec3f, 4> expected{{
      {0.0F, -1.0F, 0.0F},
      {1.0F, 0.0F, 0.0F},
      {0.0F, 1.0F, 0.0F},
      {-1.0F, 0.0F, 0.0F},
  }};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    const Vec3f axis = handAxis(static_cast<float>(i) * kPi / 2.0F);
    EXPECT_NEAR(axis.x, expected[i].x, 1.0e-6F);
    EXPECT_NEAR(axis.y, expected[i].y, 1.0e-6F);
    EXPECT_NEAR(dot(axis, handTangent(static_cast<float>(i) * kPi / 2.0F)),
                0.0F, 1.0e-6F);
  }
}

TEST(AngleState, PreservesTurnsAndPhase) {
  LogicalAngle angle{};
  angle.set(0.25);
  constexpr std::int64_t crossings = 1'000'000;
  for (std::int64_t i = 0; i < crossings; ++i) {
    angle.advance(kTwoPi);
  }
  EXPECT_EQ(angle.turns, crossings);
  EXPECT_NEAR(angle.phase, 0.25F, 1.0e-5F);
  EXPECT_NEAR(angle.value(),
              static_cast<double>(crossings) * kTwoPi + 0.25,
              0.2);
}

TEST(Configuration, RejectsInconsistentMassProperties) {
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].inertiaPivotKgM2 = 1.0e-15F;
  EXPECT_FALSE(engine.configure(config));
  EXPECT_EQ(engine.snapshot().lastError, ErrorCode::InvalidConfiguration);
}

TEST(Configuration, CoreStateFitsPcBudget) {
  PhysicalDialEngine engine;
  EXPECT_LE(engine.coreStateBytes(), 32U * 1024U);
}

TEST(Configuration, DefaultEnvelopeSatisfiesSubstepPolicy) {
  const EngineConfig config = makeDefaultConfig();
  EXPECT_LE(maximumDimensionlessSubstep(config), 0.25F);
  PhysicalDialEngine engine;
  EXPECT_TRUE(engine.configure(config));
}

TEST(Configuration, RejectsStiffnessBeyondFourSubsteps) {
  EngineConfig config = makeDefaultConfig();
  config.hands[0].touchKpNmPerRad *= 100.0F;
  EXPECT_GT(maximumDimensionlessSubstep(config), 0.25F);
  PhysicalDialEngine engine;
  EXPECT_FALSE(engine.configure(config));
}

TEST(BaseKinematics, OffsetCorrectionMatchesRigidBodyFormula) {
  const Vec3f force{1.0F, -2.0F, 3.0F};
  const Vec3f omega{0.0F, 0.0F, 4.0F};
  const Vec3f alpha{0.0F, 0.0F, 2.0F};
  const Vec3f offset{0.01F, 0.0F, 0.0F};
  const Vec3f result =
      effectiveAccelerationAtCenter(force, omega, alpha, offset);
  EXPECT_NEAR(result.x, -1.16F, 1.0e-6F);
  EXPECT_NEAR(result.y, 2.02F, 1.0e-6F);
  EXPECT_NEAR(result.z, -3.0F, 1.0e-6F);
}

}  // namespace
}  // namespace dial
