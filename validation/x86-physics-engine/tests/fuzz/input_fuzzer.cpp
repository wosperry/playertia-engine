#include "dial/test_support.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size) {
  if (size < 32U) {
    return 0;
  }
  dial::PhysicalDialEngine engine;
  dial::StepInput input = dial::test::makeInput();
  auto readFloat = [&](std::size_t offset) {
    float value{};
    if (offset + sizeof(float) <= size) {
      std::memcpy(&value, data + offset, sizeof(float));
    }
    return value;
  };
  const dial::Vec3f acceleration{readFloat(0), readFloat(4), readFloat(8)};
  const dial::Vec3f omegaStart{readFloat(12), readFloat(16), readFloat(20)};
  const dial::Vec3f omegaEnd{readFloat(24), readFloat(28),
                             size >= 36U ? readFloat(32) : 0.0F};
  input.touchTargets[0].active = (data[size - 1U] & 1U) != 0U;
  input.touchTargets[0].angleUnwrappedRad =
      size >= 40U ? readFloat(36) : 0.0F;
  input.contactCount = (data[size - 1U] & 2U) != 0U ? 1U : 0U;
  input.contacts[0] = {0, -1, {1.0F, 0.0F}, {}, readFloat(0),
                       readFloat(4), 0.0F, 1U, true};
  (void)engine.step(
      dial::test::makeBase(0U, acceleration, omegaStart, omegaEnd), input);
  (void)dial::test::finiteSnapshot(engine.snapshot());
  return 0;
}

