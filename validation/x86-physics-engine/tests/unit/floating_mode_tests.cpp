#include "dial/test_support.hpp"

#include <gtest/gtest.h>

#if defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace dial {
namespace {

TEST(FloatingPoint, FtzDazPathRemainsFinite) {
#if defined(__SSE__)
  const unsigned int original = _mm_getcsr();
  constexpr unsigned int kDaz = 1U << 6U;
  constexpr unsigned int kFtz = 1U << 15U;
  _mm_setcsr(original | kDaz | kFtz);
#endif
  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  config.hands[0].coulombFrictionNm = 1.0e-38F;
  config.hands[0].viscousDampingNmSecPerRad = 1.0e-38F;
  ASSERT_TRUE(engine.configure(config));
  engine.reset({}, {1.0e-30F, 0.0F, 0.0F}, {});
  StepInput input = test::makeInput();
  for (std::uint64_t step = 0; step < 10'000U; ++step) {
    ASSERT_TRUE(
        engine.step(test::makeBase(step * 10'000U, {}), input));
    ASSERT_TRUE(test::finiteSnapshot(engine.snapshot()));
  }
#if defined(__SSE__)
  _mm_setcsr(original);
#endif
}

}  // namespace
}  // namespace dial

