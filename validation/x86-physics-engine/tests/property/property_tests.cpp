#include "dial/reference.hpp"
#include "dial/test_support.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace dial {
namespace {

std::uint64_t propertyCases() {
  const char* raw = std::getenv("DIAL_PROPERTY_CASES");
  if (raw == nullptr) {
    return 10'000U;
  }
  return std::max<std::uint64_t>(1U, std::strtoull(raw, nullptr, 10));
}

TEST(Properties, RandomCoordinateInvariants) {
  test::XorShift64 random(0x71f15eedULL);
  const std::uint64_t cases = propertyCases();
  for (std::uint64_t i = 0; i < cases; ++i) {
    Quatf q{random.uniform(-1.0F, 1.0F),
            random.uniform(-1.0F, 1.0F),
            random.uniform(-1.0F, 1.0F),
            random.uniform(-1.0F, 1.0F)};
    q = normalized(q);
    const Vec3f vector{random.uniform(-100.0F, 100.0F),
                       random.uniform(-100.0F, 100.0F),
                       random.uniform(-100.0F, 100.0F)};
    const Vec3f roundTrip =
        rotateWorldToBody(q, rotateBodyToWorld(q, vector));
    ASSERT_NEAR(roundTrip.x, vector.x, 0.01F);
    ASSERT_NEAR(roundTrip.y, vector.y, 0.01F);
    ASSERT_NEAR(roundTrip.z, vector.z, 0.01F);
    ASSERT_NEAR(norm(gravityBody(q)), 9.80665F, 0.001F);
  }
}

TEST(Properties, DampingNeverAddsPower) {
  test::XorShift64 random(0xdadeULL);
  EngineConfig config = makeDefaultConfig();
  PhysicalDialEngine engine;
  ASSERT_TRUE(engine.configure(config));
  StepInput input = test::makeInput();
  for (TimeTarget& target : input.timeTargets) {
    target.valid = false;
  }
  const std::uint64_t cases = std::min<std::uint64_t>(propertyCases(), 20'000U);
  for (std::uint64_t i = 0; i < cases; ++i) {
    const float omega = random.uniform(-60.0F, 60.0F);
    engine.reset({}, {omega, 0.0F, 0.0F}, {});
    input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
    input.eventCount = 1;
    ASSERT_TRUE(engine.step(test::makeBase(0U, {}), input));
    const auto& hand = engine.snapshot().hands[0];
    ASSERT_LE(hand.torque.dampingNm * omega, 1.0e-12F);
    ASSERT_TRUE(test::finiteSnapshot(engine.snapshot()));
  }
}

TEST(Properties, RandomValidTrajectoriesRemainFiniteAndBounded) {
  test::XorShift64 random(0x5eedc0deULL);
  const std::uint64_t trajectories =
      std::min<std::uint64_t>(propertyCases() / 20U + 1U, 5'000U);
  for (std::uint64_t trajectory = 0; trajectory < trajectories;
       ++trajectory) {
    PhysicalDialEngine engine;
    const std::array<double, kHandCount> angles{
        random.uniform(-kPi, kPi), random.uniform(-kPi, kPi),
        random.uniform(-kPi, kPi)};
    const std::array<float, kHandCount> velocities{
        random.uniform(-20.0F, 20.0F),
        random.uniform(-20.0F, 20.0F),
        random.uniform(-20.0F, 20.0F)};
    engine.reset(angles, velocities, {});
    StepInput input = test::makeInput();
    input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
    input.eventCount = 1;
    for (std::uint64_t step = 0; step < 20U; ++step) {
      input.eventCount = step == 0U ? 1U : 0U;
      const Vec3f acceleration{
          random.uniform(-30.0F, 30.0F),
          random.uniform(-30.0F, 30.0F),
          random.uniform(-30.0F, 30.0F)};
      const Vec3f omega{
          random.uniform(-5.0F, 5.0F),
          random.uniform(-5.0F, 5.0F),
          random.uniform(-5.0F, 5.0F)};
      ASSERT_TRUE(engine.step(
          test::makeBase(step * 10'000U, acceleration, omega, omega),
          input));
      ASSERT_TRUE(test::finiteSnapshot(engine.snapshot()));
      for (const HandSnapshot& hand : engine.snapshot().hands) {
        ASSERT_LE(std::abs(hand.angularVelocityRadSec), 80.001F);
      }
    }
  }
}

TEST(Properties, FlexibleMassMatrixStaysPositive) {
  test::XorShift64 random(0xf1e1b1eULL);
  const FlexibleDefinition flex = makeDefaultConfig().hands[1].flex;
  const std::uint64_t cases = propertyCases();
  for (std::uint64_t i = 0; i < cases; ++i) {
    const long double beta = random.uniform(-kPi, kPi);
    const auto mass = reference::flexibleMassMatrix(beta, flex);
    ASSERT_GT(mass.m11, 0.0L);
    ASSERT_GT(mass.m22, 0.0L);
    ASSERT_GT(mass.determinant(), 1.0e-22L);
  }
}

}  // namespace
}  // namespace dial

