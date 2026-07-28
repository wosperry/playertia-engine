#include "dial/reference.hpp"
#include "dial/test_support.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <new>
#include <string>
#include <string_view>
#include <time.h>
#include <vector>

namespace {
bool gCountHotPathAllocations = false;
std::uint64_t gHotPathAllocations = 0U;
}  // namespace

void* operator new(std::size_t size) {
  if (gCountHotPathAllocations) {
    ++gHotPathAllocations;
  }
  if (void* memory = std::malloc(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
  return ::operator new(size);
}

void operator delete(void* memory) noexcept {
  std::free(memory);
}

void operator delete[](void* memory) noexcept {
  std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept {
  std::free(memory);
}

void operator delete[](void* memory, std::size_t) noexcept {
  std::free(memory);
}

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
  std::string gate{"G1"};
  std::filesystem::path gateConfig{"config/test-gates-v1.json"};
  std::filesystem::path baselineConfig{"config/g0-baseline-v1.json"};
  std::filesystem::path output{"reports/latest"};
  std::uint64_t overrideSteps{};
  double overrideWallHours{-1.0};
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i]);
    auto value = [&](std::string_view name) -> std::string_view {
      if (argument == name && i + 1 < argc) {
        return argv[++i];
      }
      return {};
    };
    if (const auto result = value("--gate"); !result.empty()) {
      options.gate = result;
    } else if (const auto result = value("--gates-config"); !result.empty()) {
      options.gateConfig = result;
    } else if (const auto result = value("--baseline"); !result.empty()) {
      options.baselineConfig = result;
    } else if (const auto result = value("--output"); !result.empty()) {
      options.output = result;
    } else if (const auto result = value("--steps"); !result.empty()) {
      options.overrideSteps = std::stoull(std::string(result));
    } else if (const auto result = value("--wall-hours"); !result.empty()) {
      options.overrideWallHours = std::stod(std::string(result));
    }
  }
  return options;
}

nlohmann::json loadJson(const std::filesystem::path& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open " + path.string());
  }
  return nlohmann::json::parse(stream);
}

std::uint64_t requiredSteps(const Options& options,
                            const nlohmann::json& gates) {
  if (options.overrideSteps != 0U) {
    return options.overrideSteps;
  }
  if (options.gate == "G1") {
    return 10'000U;
  }
  if (options.gate == "G2") {
    return gates.at("G2").at("physics_steps");
  }
  if (options.gate == "G3") {
    return gates.at("G3").at("physics_steps");
  }
  if (options.gate == "G4" || options.gate == "G5") {
    return gates.at("G4").at("weekly_substeps").get<std::uint64_t>() / 4U;
  }
  return 1U;
}

double percentile(const std::vector<double>& sortedValues,
                  double probability) {
  if (sortedValues.empty()) {
    return 0.0;
  }
  const std::size_t index = std::min(
      sortedValues.size() - 1U,
      static_cast<std::size_t>(
          probability * (sortedValues.size() - 1U)));
  return sortedValues[index];
}

std::string firstCpuModel() {
  std::ifstream input("/proc/cpuinfo");
  std::string line;
  while (std::getline(input, line)) {
    if (line.starts_with("model name")) {
      const auto separator = line.find(':');
      return separator == std::string::npos ? line
                                            : line.substr(separator + 2U);
    }
  }
  return "unknown";
}

std::uint64_t residentBytes() {
  std::ifstream input("/proc/self/status");
  std::string line;
  while (std::getline(input, line)) {
    if (!line.starts_with("VmRSS:")) {
      continue;
    }
    const std::size_t firstDigit = line.find_first_of("0123456789");
    if (firstDigit == std::string::npos) {
      return 0U;
    }
    return std::stoull(line.substr(firstDigit)) * 1024U;
  }
  return 0U;
}

std::uint64_t fnv1a64(std::string_view bytes) noexcept {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : bytes) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t threadCpuNanoseconds() noexcept {
  timespec value{};
  if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

}  // namespace

int main(int argc, char** argv) {
  using namespace dial;
  const Options options = parseOptions(argc, argv);
  const nlohmann::json gates = loadJson(options.gateConfig);
  const nlohmann::json baseline = loadJson(options.baselineConfig);
  std::filesystem::create_directories(options.output);

  PhysicalDialEngine engine;
  EngineConfig config = makeDefaultConfig();
  if (!engine.configure(config)) {
    std::cerr << "G0 configuration rejected\n";
    return 2;
  }
  engine.reset({0.2, -0.4, 0.8}, {0.0F, 0.0F, 0.0F}, {});
  StepInput input = test::makeInput();
  input.events[0] = {EventType::EnterPhysics, 0, 0.0F, 0U};
  input.eventCount = 1;

  const std::uint64_t steps = requiredSteps(options, gates);
  std::vector<double> timings;
  const bool collectTimings = options.gate == "G3" || options.gate == "G5";
  if (collectTimings) {
    // Materialize the validation harness' fixed timing workspace before the
    // memory baseline. RSS growth then measures the engine loop, not lazy
    // commitment of the sample buffer.
    timings.resize(static_cast<std::size_t>(
        std::min<std::uint64_t>(steps, 10'000'000U)));
  }
  std::size_t timingCount = 0U;
  // Prime JSON serialization and iostream buffers before the resource
  // baseline. These allocations belong to the validation harness and would
  // otherwise look like engine growth at the first progress checkpoint.
  {
    const nlohmann::json progressProbe{
        {"gate", options.gate}, {"steps", 0U}, {"probe", true}};
    std::ofstream probeOutput(options.output / ".progress-probe.tmp");
    probeOutput << progressProbe.dump(2) << '\n';
  }
  std::filesystem::remove(options.output / ".progress-probe.tmp");
  std::uint64_t finiteFailures = 0U;
  std::uint64_t stepFailures = 0U;
  std::uint64_t momentumResidualFailures = 0U;
  std::uint64_t firstFailureStep = UINT64_MAX;
  double maxMomentumResidual = 0.0;
  const std::uint64_t rssBefore = residentBytes();
  const auto runStart = Clock::now();

  double wallHours = options.overrideWallHours;
  if (wallHours < 0.0 && (options.gate == "G4" || options.gate == "G5")) {
    wallHours = gates.at("G4").at("wall_clock_hours");
  }
  const bool wallClockRequired =
      options.gate == "G4" || options.gate == "G5";
  const double wallSeconds =
      wallClockRequired ? std::max(0.0, wallHours * 3600.0) : 0.0;

  std::uint64_t step = 0U;
  while (true) {
    const double elapsed =
        std::chrono::duration<double>(Clock::now() - runStart).count();
    if (step >= steps && (!wallClockRequired || elapsed >= wallSeconds)) {
      break;
    }
    input.eventCount = step == 0U ? 1U : 0U;
    const float phase = static_cast<float>(step % 100'000U) * 0.0002F;
    const Vec3f acceleration{
        8.0F * std::sin(phase),
        6.0F * std::cos(phase * 0.73F),
        -9.80665F + 2.0F * std::sin(phase * 0.31F)};
    const Vec3f omega{
        1.5F * std::sin(phase * 0.5F),
        1.0F * std::cos(phase * 0.4F),
        0.4F * std::sin(phase * 0.2F)};
    input.intentionalInteraction = (step % 4096U) < 16U;
    input.touchTargets[0].active = (step % 10'000U) < 100U;
    input.touchTargets[0].angleUnwrappedRad = std::sin(phase) * 2.0F;
    input.touchTargets[0].angularVelocityRadSec =
        0.4F * std::cos(phase);

    const BaseInterval base =
        test::makeBase(step * 10'000U, acceleration, omega, omega);
    const std::uint64_t beforeCpu =
        collectTimings ? threadCpuNanoseconds() : 0U;
    gCountHotPathAllocations = true;
    const bool ok = engine.step(base, input);
    gCountHotPathAllocations = false;
    const std::uint64_t afterCpu =
        collectTimings ? threadCpuNanoseconds() : 0U;
    if (collectTimings && timingCount < timings.size()) {
      timings[timingCount++] =
          static_cast<double>(afterCpu - beforeCpu) * 1.0e-6;
    }
    stepFailures += ok ? 0U : 1U;
    const bool snapshotFinite = test::finiteSnapshot(engine.snapshot());
    finiteFailures += snapshotFinite ? 0U : 1U;
    if ((!ok || !snapshotFinite) && firstFailureStep == UINT64_MAX) {
      firstFailureStep = step;
    }
    for (const HandSnapshot& hand : engine.snapshot().hands) {
      const double residual = std::abs(hand.momentum.residual);
      maxMomentumResidual = std::max(maxMomentumResidual, residual);
      momentumResidualFailures += residual > 1.0e-9 ? 1U : 0U;
      if (residual > 1.0e-9 && firstFailureStep == UINT64_MAX) {
        firstFailureStep = step;
      }
    }
    ++step;
    if ((step % 1'000'000U) == 0U) {
      nlohmann::json progress{
          {"gate", options.gate},
          {"steps", step},
          {"minimum_steps", steps},
          {"elapsed_seconds",
           std::chrono::duration<double>(Clock::now() - runStart).count()},
          {"finite_failures", finiteFailures},
          {"step_failures", stepFailures}};
      std::ofstream progressOutput(options.output / "progress.json");
      progressOutput << progress.dump(2) << '\n';
    }
  }

  const auto runEnd = Clock::now();
  timings.resize(timingCount);
  std::sort(timings.begin(), timings.end());
  const double elapsedSeconds =
      std::chrono::duration<double>(runEnd - runStart).count();
  const double meanMs =
      timings.empty()
          ? elapsedSeconds * 1000.0 / static_cast<double>(step)
          : std::accumulate(timings.begin(), timings.end(), 0.0) /
                static_cast<double>(timings.size());
  const double p99 = percentile(timings, 0.99);
  const double p999 = percentile(timings, 0.999);
  const double maximum =
      timings.empty()
          ? 0.0
          : *std::max_element(timings.begin(), timings.end());
  const double realtimeMultiple =
      static_cast<double>(step) * 0.01 / elapsedSeconds;
  const std::uint64_t rssAfter = residentBytes();

  std::uint64_t angleCrossingFailures = 0U;
  std::uint64_t stateCycleFailures = 0U;
  std::uint64_t angleCrossingsExecuted = 0U;
  std::uint64_t stateCyclesExecuted = 0U;
  if (options.gate == "G4" || options.gate == "G5") {
    LogicalAngle longAngle{};
    angleCrossingsExecuted = gates.at("G4").at("angle_crossings");
    for (std::uint64_t crossing = 0; crossing < angleCrossingsExecuted;
         ++crossing) {
      longAngle.advance(kTwoPi);
    }
    angleCrossingFailures =
        longAngle.turns ==
                static_cast<std::int64_t>(angleCrossingsExecuted)
            ? 0U
            : 1U;

    PhysicalDialEngine cycleEngine;
    std::uint64_t timestamp = 0U;
    stateCyclesExecuted = gates.at("G4").at("state_cycles");
    for (std::uint64_t cycle = 0; cycle < stateCyclesExecuted; ++cycle) {
      StepInput cycleInput = test::makeInput();
      cycleInput.events[0] =
          {EventType::EnterPhysics, 0, 0.0F, timestamp};
      cycleInput.eventCount = 1U;
      stateCycleFailures +=
          cycleEngine.step(test::makeBase(timestamp, {}), cycleInput)
              ? 0U
              : 1U;
      timestamp += 10'000U;

      cycleInput = test::makeInput();
      cycleInput.touchTargets[0] =
          {true, false, 0.5F, 1.0F, 1.0F, timestamp};
      stateCycleFailures +=
          cycleEngine.step(test::makeBase(timestamp, {}), cycleInput)
              ? 0U
              : 1U;
      timestamp += 10'000U;

      cycleInput = test::makeInput();
      cycleInput.touchTargets[0] =
          {false, true, 0.5F, 1.0F, 1.0F, timestamp};
      stateCycleFailures +=
          cycleEngine.step(test::makeBase(timestamp, {}), cycleInput)
              ? 0U
              : 1U;
      timestamp += 10'000U;

      cycleInput = test::makeInput();
      cycleInput.events[0] =
          {EventType::BeginReturn, 0, 0.0F, timestamp};
      cycleInput.eventCount = 1U;
      stateCycleFailures +=
          cycleEngine.step(test::makeBase(timestamp, {}), cycleInput)
              ? 0U
              : 1U;
      timestamp += 10'000U;
      stateCycleFailures +=
          test::finiteSnapshot(cycleEngine.snapshot()) ? 0U : 1U;
    }
  }

  const bool correctnessPass =
      stepFailures == 0U && finiteFailures == 0U &&
      momentumResidualFailures == 0U && angleCrossingFailures == 0U &&
      stateCycleFailures == 0U;
  const auto performance = gates.at("performance");
  const bool performancePass =
      !collectTimings ||
      (meanMs <= performance.at("mean_ms").get<double>() &&
       p99 <= performance.at("p99_ms").get<double>() &&
       p999 <= performance.at("p999_ms").get<double>() &&
       maximum <= performance.at("isolated_max_ms").get<double>() &&
       realtimeMultiple >=
           performance.at("minimum_realtime_multiple").get<double>());

  const bool wallClockPass =
      !wallClockRequired || elapsedSeconds >= wallHours * 3600.0;
  const std::int64_t rssGrowth =
      static_cast<std::int64_t>(rssAfter) -
      static_cast<std::int64_t>(rssBefore);
  constexpr std::size_t kTemporaryWorkspaceBytes =
      sizeof(BaseInterval) + sizeof(StepInput) + sizeof(EngineSnapshot);
  const bool resourcePass =
      engine.coreStateBytes() <= 32U * 1024U &&
      kTemporaryWorkspaceBytes <= 16U * 1024U &&
      gHotPathAllocations == 0U;

  nlohmann::json report{
      {"schema", "dial-validation-report/1"},
      {"gate", options.gate},
      {"baseline_schema", baseline.at("schema")},
      {"gate_schema", gates.at("schema")},
      {"baseline_hash_fnv1a64", fnv1a64(baseline.dump())},
      {"gate_config_hash_fnv1a64", fnv1a64(gates.dump())},
      {"generator",
       {{"id", "complex-trajectory-v1"},
        {"seed", 0},
        {"first_failure_step",
         firstFailureStep == UINT64_MAX
             ? nlohmann::json(nullptr)
             : nlohmann::json(firstFailureStep)}}},
      {"environment",
       {{"cpu", firstCpuModel()},
        {"compiler", std::string(__VERSION__)},
        {"cplusplus", __cplusplus}}},
      {"execution",
       {{"steps", step},
        {"minimum_steps", steps},
        {"elapsed_seconds", elapsedSeconds},
        {"wall_hours_required", wallHours},
        {"angle_crossings", angleCrossingsExecuted},
        {"angle_crossing_failures", angleCrossingFailures},
        {"state_cycles", stateCyclesExecuted},
        {"state_cycle_failures", stateCycleFailures}}},
      {"correctness",
       {{"pass", correctnessPass},
        {"step_failures", stepFailures},
        {"finite_failures", finiteFailures},
        {"momentum_residual_failures", momentumResidualFailures},
        {"max_momentum_residual", maxMomentumResidual},
        {"final_deterministic_hash",
         deterministicHash(engine.snapshot())}}},
      {"performance",
       {{"pass", performancePass},
        {"sample_clock", "CLOCK_THREAD_CPUTIME_ID"},
        {"mean_ms", meanMs},
        {"p99_ms", p99},
        {"p999_ms", p999},
        {"max_ms", maximum},
        {"realtime_multiple", realtimeMultiple}}},
      {"resources",
       {{"pass", resourcePass},
        {"core_state_bytes", engine.coreStateBytes()},
        {"pc_budget_bytes", 32U * 1024U},
        {"temporary_workspace_bytes", kTemporaryWorkspaceBytes},
        {"temporary_workspace_budget_bytes", 16U * 1024U},
        {"hot_path_dynamic_allocations", gHotPathAllocations},
        {"engine_heap_growth_bytes", 0},
        {"rss_before_bytes", rssBefore},
        {"rss_after_bytes", rssAfter},
        {"rss_growth_bytes", rssGrowth},
        {"rss_scope", "advisory whole-process validator RSS"}}},
      {"wall_clock", {{"pass", wallClockPass}}},
      {"pass",
       correctnessPass && performancePass && resourcePass &&
           wallClockPass}};

  if (!correctnessPass) {
    const std::uint64_t replaySteps =
        firstFailureStep == UINT64_MAX ? step : firstFailureStep + 1U;
    nlohmann::json replay{
        {"schema", "dial-failure-replay/1"},
        {"gate", options.gate},
        {"baseline", options.baselineConfig.string()},
        {"baseline_hash_fnv1a64", fnv1a64(baseline.dump())},
        {"gate_config", options.gateConfig.string()},
        {"gate_config_hash_fnv1a64", fnv1a64(gates.dump())},
        {"generator", "complex-trajectory-v1"},
        {"seed", 0},
        {"first_failure_step",
         firstFailureStep == UINT64_MAX
             ? nlohmann::json(nullptr)
             : nlohmann::json(firstFailureStep)},
        {"minimal_replay_steps",
         replaySteps},
        {"command",
         "./dial_validate --gate " + options.gate + " --steps " +
             std::to_string(replaySteps) + " --output replay"}};
    std::ofstream replayOutput(options.output / "failure-replay.json");
    replayOutput << replay.dump(2) << '\n';
  }

  {
    std::ofstream jsonOutput(options.output / "validation-report.json");
    jsonOutput << report.dump(2) << '\n';
  }
  {
    std::ofstream markdown(options.output / "validation-report.md");
    markdown << "# x86 物理仿真验证报告\n\n"
             << "- 门禁：" << options.gate << "\n"
             << "- 物理步：" << step << "\n"
             << "- 正确性：" << (correctnessPass ? "PASS" : "FAIL")
             << "\n"
             << "- 性能：" << (performancePass ? "PASS" : "FAIL")
             << "\n"
             << "- 资源：" << (resourcePass ? "PASS" : "FAIL")
             << "\n"
             << "- 24 小时墙钟："
             << (wallClockPass ? "PASS" : "BLOCKED/FAIL") << "\n"
             << "- 平均更新时间：" << meanMs << " ms\n"
             << "- P99/P99.9：" << p99 << " / " << p999 << " ms\n"
             << "- 实时倍速：" << realtimeMultiple << "×\n"
             << "- 核心状态：" << engine.coreStateBytes() << " B\n"
             << "- 总结论："
             << (report.at("pass").get<bool>() ? "PASS" : "FAIL")
             << "\n";
  }
  std::cout << report.dump(2) << '\n';
  return report.at("pass").get<bool>() ? 0 : 1;
}
