#include "dial/test_support.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace dial {
namespace {

TEST(FaultPolicy, RejectsNonFiniteInputWithoutStateAdvance) {
  PhysicalDialEngine engine;
  engine.reset({}, {}, {});
  const EngineSnapshot before = engine.snapshot();
  BaseInterval base = test::makeBase(0U, {});
  base.quarter[2].effectiveAccelImpulseMps.x =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(engine.step(base, test::makeInput()));
  EXPECT_EQ(engine.snapshot().rejectedIntervals, 1U);
  EXPECT_EQ(engine.snapshot().hands[0].angleUnwrappedRad,
            before.hands[0].angleUnwrappedRad);
}

TEST(FaultPolicy, RejectsDuplicateOrOutOfOrderInterval) {
  PhysicalDialEngine engine;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), test::makeInput()));
  EXPECT_FALSE(engine.step(test::makeBase(0U, {}), test::makeInput()));
  EXPECT_EQ(engine.snapshot().lastError, ErrorCode::InvalidInterval);
}

TEST(FaultPolicy, DeclaredGapEntersDegradedState) {
  PhysicalDialEngine engine;
  ASSERT_TRUE(engine.step(test::makeBase(0U, {}), test::makeInput()));
  ASSERT_TRUE(engine.step(
      test::makeBase(30'000U, {}, {}, {}, {0.0F, 0.0F, -9.80665F},
                     QualityGap),
      test::makeInput()));
  EXPECT_EQ(engine.snapshot().globalMode, GlobalMode::Degraded);
  EXPECT_EQ(engine.snapshot().degradationCount, 1U);
  EXPECT_TRUE(test::finiteSnapshot(engine.snapshot()));
}

TEST(FaultPolicy, RejectsOversizedInputCollections) {
  PhysicalDialEngine engine;
  StepInput input = test::makeInput();
  input.eventCount = kMaxEvents + 1U;
  EXPECT_FALSE(engine.step(test::makeBase(0U, {}), input));
  input = test::makeInput();
  input.contactCount = kMaxContacts + 1U;
  EXPECT_FALSE(engine.step(test::makeBase(0U, {}), input));
}

TEST(FaultPolicy, ExtremeLegalInputRemainsFinite) {
  PhysicalDialEngine engine;
  engine.reset({kPi, -kPi, 1000.0}, {79.0F, -79.0F, 50.0F},
               {20.0F, -20.0F, 30.0F});
  StepInput input = test::makeInput();
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;
  for (std::uint64_t step = 0; step < 10'000U; ++step) {
    input.eventCount = step == 0U ? 1U : 0U;
    const float sign = (step & 1U) == 0U ? 1.0F : -1.0F;
    ASSERT_TRUE(engine.step(
        test::makeBase(step * 10'000U,
                       {156.0F * sign, -156.0F * sign, 0.0F},
                       {20.0F, -20.0F, 30.0F},
                       {-20.0F, 20.0F, -30.0F}),
        input));
    ASSERT_TRUE(test::finiteSnapshot(engine.snapshot()));
  }
}

}  // namespace
}  // namespace dial

