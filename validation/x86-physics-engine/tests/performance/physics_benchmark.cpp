#include "dial/test_support.hpp"

#include <benchmark/benchmark.h>

namespace dial {
namespace {

static void LoadFull(benchmark::State& state) {
  PhysicalDialEngine engine;
  StepInput input = test::makeInput();
  input.intentionalInteraction = true;
  input.touchTargets[0] = {true, false, 0.7F, 0.5F, 1.0F, 0U};
  std::uint64_t sequence = 0;
  for (auto _ : state) {
    const BaseInterval base =
        test::makeBase(sequence++ * 10'000U, {2.0F, 3.0F, -9.0F},
                       {0.2F, -0.1F, 0.3F}, {0.2F, -0.1F, 0.3F});
    benchmark::DoNotOptimize(engine.step(base, input));
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(LoadFull)->MinWarmUpTime(1.0)->MinTime(2.0);

static void ScaleObjectsEquivalent(benchmark::State& state) {
  const int repetitions = static_cast<int>(state.range(0));
  PhysicalDialEngine engine;
  StepInput input = test::makeInput();
  std::uint64_t sequence = 0;
  for (auto _ : state) {
    for (int i = 0; i < repetitions; ++i) {
      const BaseInterval base =
          test::makeBase(sequence++ * 10'000U, {1.0F, 2.0F, 0.0F});
      benchmark::DoNotOptimize(engine.step(base, input));
    }
  }
  state.SetItemsProcessed(state.iterations() * repetitions * 3);
}
BENCHMARK(ScaleObjectsEquivalent)->Arg(1)->Arg(2)->Arg(4)->Arg(8);

}  // namespace
}  // namespace dial
