# Playertia Engine

English | [简体中文](README.zh-CN.md)

Playertia Engine is a portable, compact physics interaction engine for small
screens and devices equipped with IMUs, touch input, audio, and haptics.

It is not intended to compete with general-purpose engines such as Unity.
Instead, it focuses on small, believable physical worlds made from a limited
number of interactive objects:

- device tilt, rotation, translation, and rapid motion affect scenes physically;
- touch and button input enter the simulation as forces, constraints, or impulses;
- objects have mass, centers of mass, inertia, damping, friction, and flexibility;
- fixed-timestep physics remains independent of the display frame rate;
- collisions emit events that renderers, audio systems, and haptic systems consume;
- the same core can run on ESP32-S3, desktop, and mobile platforms.

The first target scenes are:

1. a physical watch whose hands can be moved, released, swung, and returned to time;
2. a live background containing three or four freely moving, audible marbles;
3. a miniature marble maze with walls, holes, goals, scale, and visual depth;
4. future physical toys such as springs, pendulums, gyroscopes, ratchets, and wind-up mechanisms.

## Design Principles

1. **Build scenes before abstractions.** Shared engine functionality emerges only after real reuse.
2. **Start from physics.** Reference frames, degrees of freedom, mass properties, and forces come before animation rules.
3. **Stay compact and deterministic.** Fixed capacities, fixed timesteps, replayable input, and no allocation on the hot path.
4. **Separate simulation from presentation.** Rendering, audio, and haptics consume state and events without owning physics.
5. **Keep scenes isolated.** Adding marbles or a maze must not alter the no-contact behavior of watch hands.
6. **Keep the core portable.** Platform layers provide sensors, clocks, touch, rendering, and lifecycle integration.
7. **Avoid premature generality.** There is currently no general ECS, scripting language, editor, or full 3D rigid-body system.
8. **Maintain a narrow public boundary.** Internal C++ models may evolve while applications depend on a versioned C ABI.
9. **Test integration quality.** Examples use only public interfaces and double as package integration tests.

## Repository Layout

```text
.
├── CMakeLists.txt                 # Unified engine and SDK build entry point
├── engine/
│   ├── physics/                   # Production physics core
│   └── runtime/                   # Planned shared clock, lifecycle, and event boundary
├── sdk/
│   ├── c/                         # Versioned, opaque, cross-language C ABI
│   ├── packages/                  # Android, Apple, and HarmonyOS packages
│   └── bindings/                  # Framework bindings added only when needed
├── scenes/
│   ├── physical-watch/            # Current primary scene
│   ├── free-balls/                # Planned free-marble scene
│   └── ball-maze/                 # Planned miniature maze
├── platforms/                     # ESP32-S3 and other device adapters
├── examples/                      # Cross-platform examples using only the public SDK
└── validation/
    ├── requirements/              # Independent validation requirements
    └── x86-physics-engine/        # Oracle, tests, fuzzing, benchmarks, and reports
```

## Current Status

Implemented:

- an independent C++20 three-hand physics core;
- two rigid hands and one two-segment flexible hand;
- world gravity and translating/rotating dial-base models;
- touch, TIME_REF, and mechanical, animated, and hybrid return;
- generic contact constraints and energy, impulse, torque, and safety ledgers;
- a separately buildable, versioned C ABI;
- an independent high-precision x86 oracle and G0–G5 validation system.

Not yet implemented:

- ESP32-S3 firmware and the QMI8658 driver;
- AMOLED rendering and material presentation;
- touch hit testing and gesture recognition;
- free-marble and maze geometry;
- audio, haptics, and platform lifecycle adapters;
- native Android, Apple, and HarmonyOS packages.

## Entry Points

- [Production physics core](engine/physics/README.md)
- [SDK boundary](sdk/README.md)
- [Runtime boundary](engine/runtime/README.md)
- [Scenes](scenes/README.md)
- [Physical watch](scenes/physical-watch/README.md)
- [Device adapters](platforms/README.md)
- [Public examples](examples/README.md)
- [Validation](validation/README.md)

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

The default configuration builds the production physics core, the Playertia C
ABI, and its C smoke test. The full x86 validation suite remains independently
buildable from `validation/x86-physics-engine`.

## License

Playertia Engine is currently proprietary software. All rights are reserved,
and no permission to use, copy, modify, or distribute the project is granted
without prior written authorization. See [LICENSE](LICENSE).
