# Change: Refactor IK Solver for Performance and Robustness (refactor-ik-solver-restructure)

## Why

The current `SQPIKSolver` implements the Sequential Quadratic Programming (SQP) algorithm and works in production, but several implementation-level choices limit performance and robustness. These bottlenecks prevent the project meeting higher performance targets and consistent solve success rates required by industrial robot control (e.g., microsecond-level solves, near-100% success for multistart strategies).

This change aims to rework the IK solver stack to achieve:
- Zero heap allocations per SQP iteration (avoid per-iteration heap use)
- FK/Jacobian compute fusion to remove duplicated transforms
- Aggressive compiler optimizations and Eigen vectorization
- A race-style multi-start strategy with TLS-based solver pool to reach near 100% convergence success and keep total time bounded

## What Changes

- Core Optimization (Stage 1)
  - Rework `SQPIKSolver` to make the temporary matrices and vectors used in the solve loop member fields and pre-resize based on DOF. No dynamic memory allocations during iterations.
  - Modify the `DaQPSolver` wrapper to persist `lower_`, `upper_`, `x_` arrays and to avoid copying vectors to/from temporary `std::vector` in hot paths.
  - Introduce `RobotState` as a cache of global link transforms. Update `ForwardKinematics` to populate `RobotState` and update `JacobianCalculator` to accept a `RobotState` enabling `FK` + `Jacobian` fusion (no recomputation of transforms).
  - Adjust `CMakeLists.txt` (non-MSVC compilers) to use `-march=native`, set `EIGEN_NO_DEBUG`, and ensure optimized builds use vectorization and no debug checks.

- Multi-Start Strategy (Stage 2)
  - Add a new `RacingIKSolver` or an option in `SQPIKSolver` enabling a multi-start racing strategy that concurrently attempts N initial guesses (warmstart, zero config, and N-2 random samples). The fastest found solution wins and cancels other runners.
  - Implement a TLS-based solver pool (one `SQPIKSolver` + `Robot` instance per worker thread) to avoid `Robot::clone()` overhead at runtime and to support safe concurrency without creating threads per call.
  - Make `Robot` cloning cheaper by decoupling heavy state (kinematic graph data) from runtime state (cached transforms and solvers). Use a lightweight copy-on-write or explicit `clone()` semantics when a deep copy is needed.

- Tooling & Tests
  - Add performance benchmarks and new unit tests to cover zero-allocation hot path, FK/Jacobian fusion correctness, and the multi-start racing strategy correctness/time budgets.
  - Add integration tests to verify multi-threaded racing returns consistent results (same or better correctness compared to existing single-run solver).

## Impact

- Affected specs: `inverse-kinematics`, `forward-kinematics`, `jacobian-computation`, `robot-model`.
- Affected code files:
  - `core/src/inverse_kinematics.cpp` (SQPIKSolver, DaQPSolver)
  - `core/include/kinex/inverse_kinematics.h`
  - `core/src/robot.cpp` / `core/include/kinex/robot.h` (Robot cloning, ensureIKSolver)
  - `core/src/kinematics.cpp`/`forward_kinematics.cpp` (FK), `jacobian_calculator.*` (if present)
  - `CMakeLists.txt`, `cmake/Dependencies.cmake` (build flags)
- Tests & Benchmarks: `benchmarks/` and `core/tests/test_inverse_kinematics.cpp` and related tests will be updated.

## Breaking Changes
- Changing the `JacobianCalculator` to accept `RobotState` and change to FK fusion-based API is a source-compatible change if we keep the original APIs and add an overload; otherwise, it may be breaking.
- Exposing TLS-backed solver pool and racing may change how `Robot` cloning and `ensureIKSolver()` semantics work. We will avoid API-breaking changes by introducing new public APIs for multi-start options.

## Next Steps
- Draft and validate spec deltas in `openspec/changes/refactor-ik-solver-restructure/specs/`.
- Create `tasks.md` for step-by-step implementation and tests.
- Produce `design.md` describing detailed architecture decisions (data structures, thread pools, cancellation token semantics, memory layout, and DaQP integration).