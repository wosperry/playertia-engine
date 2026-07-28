#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${project_dir}/build/mutations"
report_dir="${project_dir}/reports/mutations"
mkdir -p "${build_root}" "${report_dir}"

declare -A filters=(
  [1]="RigidDynamics.EffectiveFieldTorqueHasCorrectSign"
  [2]="RigidDynamics.EffectiveFieldTorqueHasCorrectSign"
  [3]="RigidDynamics.ReducedMomentumDoesNotDoubleEulerTerm"
  [4]="RigidDynamics.ConstantDriveMatchesDiscreteMomentum"
  [5]="Properties.DampingNeverAddsPower"
  [6]="Scenarios.TimeReferenceNeverInstantlyOverwritesMechanicalAngle"
  [7]="Scenarios.ReleasePreservesDeclaredVelocity"
  [8]="FlexibleDynamics.InternalBendPreservesCyclicMomentum"
  [9]="Collision.ImpulseMomentumAndDuplicateConsumption"
  [10]="RigidDynamics.EventImpulseConsumedExactlyOnce"
)

passed=0
failed=0
for mutation in $(seq 1 10); do
  build_dir="${build_root}/${mutation}"
  cmake -S "${project_dir}" -B "${build_dir}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="${project_dir}/build/gcc-release/_deps/googletest-src" \
    -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON="${project_dir}/build/gcc-release/_deps/nlohmann_json-src" \
    -DDIAL_BUILD_BENCHMARKS=OFF \
    -DDIAL_BUILD_FUZZERS=OFF \
    -DDIAL_MUTATION_ID="${mutation}" >/dev/null
  cmake --build "${build_dir}" --target dial_tests -j2 >/dev/null
  if "${build_dir}/dial_tests" \
      --gtest_filter="${filters[$mutation]}" \
      >"${report_dir}/mutation-${mutation}.log" 2>&1; then
    echo "mutation ${mutation}: SURVIVED"
    failed=$((failed + 1))
  else
    echo "mutation ${mutation}: KILLED"
    passed=$((passed + 1))
  fi
done

printf '{"schema":"dial-mutation-report/1","killed":%d,"survived":%d,"pass":%s}\n' \
  "${passed}" "${failed}" "$([[ ${failed} -eq 0 ]] && echo true || echo false)" \
  >"${report_dir}/summary.json"

[[ ${failed} -eq 0 ]]
