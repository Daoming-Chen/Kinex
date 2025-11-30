# Tasks for refactor-ik-solver-restructure

## Context
These tasks implement the SQPIKSolver performance and robustness refactor described in `proposal.md`.

## 1. Design & Spec
- [x] 1.1 Draft `design.md` describing TLS, memory layout for zero-allocation loop, FK/Jacobian fusion and cancellation semantics
- [ ] 1.2 Add ADDED/MODIFIED requirements for `inverse-kinematics`, `forward-kinematics`, and `jacobian-computation` specs

## 2. Core Implementation
- [x] 2.1 Modify `DaQPSolver` in `core/src/inverse_kinematics.cpp` to store persistent arrays (`lower_`, `upper_`, `x_`) and provide zero-copy access
- [x] 2.2 Refactor `SQPIKSolver`:
  - [x] 2.2.1 Promote `weighted_error`, `delta`, `H`, `g` to member fields, pre-resize based on DOF
  - [x] 2.2.2 Replace Eigen temporary allocs with `noalias()` where appropriate
  - [ ] 2.2.3 Add early-exit/cancel paths to the main loop
- [x] 2.3 Implement `RobotState` caching: store global transforms per link and provide an interface for computing FK once and reusing it across Jacobian calls
- [x] 2.4 Update `JacobianCalculator` to accept `RobotState` and to optionally compute cross terms without re-evaluating transforms
- [ ] 2.5 Update `Robot::ensureIKSolver` to support TLS solver pool and add configuration for multi-start/racing

## 3. Multi-Start Racing
- [ ] 3.1 Implement `RacingIKSolver` (or extend `SQPIKSolver`) to support multiple starts with cancellation semantics
- [ ] 3.2 Implement TLS solver pool with warm-up resize for `SQPIKSolver` and `Robot` instances; track per-thread pre-allocated structures
- [ ] 3.3 Implement the initial guess generator: warm start, zero, uniform random in joint limits

## 4. Build & Tooling
- [x] 4.1 Add CMake flags to enable `-march=native` and `EIGEN_NO_DEBUG` for non-MSVC compilers
- [x] 4.2 Add environment flag for CMake to optionally enable TIMING build or `-march` settings explicitly

## 5. Testing & Benchmarks
- [x] 5.1 Unit tests: zero-allocation measurement in `JacobianCalculator` and `SQPIKSolver` hot path
- [x] 5.2 Integration tests: compare outputs of single-run solver vs racing solver, equality tolerance for converged solutions
- [x] 5.3 Benchmarks: microbench for single IK solve and multi-start constrained to 1ms total
- [ ] 5.4 Stress tests: multi-threaded solver pool with at least 8 threads, repeated solves

## 6. Validation & Documentation
- [ ] 6.1 Run `openspec validate refactor-ik-solver-restructure --strict` and fix any issues
- [ ] 6.2 Document API updates and migration guidance in `docs/` and `CHANGELOG.md`.
- [ ] 6.3 Create PR with the implementation and tests and link to this change ID

## 7. Post-Merge / Follow-Up
- [ ] 7.1 Add performance dashboards to `benchmarks/` for daily CI runs
- [ ] 7.2 Archive change once merged and the spec is updated

Notes:
- Keep the changes incremental: provide compatibility overloads where possible (e.g., new `compute()` signatures that accept `RobotState` while keeping existing ones).
- Avoid altering optimizer interfaces unless necessary to expose the necessary memory reuse.