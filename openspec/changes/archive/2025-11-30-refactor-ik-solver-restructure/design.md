# Design: IK Solver Performance Rework

## Context
The IK solver stack is composed of: `Robot` / `RobotModel` → `ForwardKinematics` → `JacobianCalculator` → `SQPIKSolver` → `DaQPSolver`.
We will preserve current behavior while enabling low-level performance improvements and a high-level multistart racing strategy.

## Goals
- Zero dynamic allocations within the SQP solve loop
- FK and Jacobian compute reuse of kinematic transforms
- 1ms multistart IK with near-100% success rate
- Thread-safety and TLS-based solver pool with minimal memory overhead

## Concepts
### RobotState
- A simple POD-like structure that stores per-link global transform (Eigen::Isometry3d or 4x4 matrix) and flags
- Populated once per call via a new `ForwardKinematics::computeRobotState(q, RobotState&)` API
- `RobotState` is passed as const reference to `JacobianCalculator::computeFromRobotState(q, RobotState&, J)`

### DaQPSolver persistent arrays
- The `DaQPSolver` wrapper will own `std::vector<double> lower_, upper_, x_` and will expose `double*` to the C API used by `daqp_quadprog` to avoid vector copying
- Reuse the same DAQPProblem object per call if problem sparsity does not change (resize at initialization)

### SQPIKSolver zero-malloc loop
- Promote these iterative variables to member fields: `Eigen::VectorXd weighted_error, delta, q, joint_limits_lower, joint_limits_upper, temp; Eigen::MatrixXd H, J, W`.
- Resize once in constructor based on dof; on solve reuse them. Avoid local temporaries.
- All matrix math should be `noalias` and use pre-sized Eigen containers to avoid allocations.

### FK/Jacobian fusion
- Provide an API on `ForwardKinematics` to compute `RobotState` and optionally a combined `computeFKAndJacobian(q, &robotState, &jacobian)` that fills the Jacobian using existing transforms.
- `JacobianCalculator` will no longer compute transforms; it will expect transforms from `RobotState`.

### Multistart Racing & TLS Solver Pool
- Implement `RacingIKSolver` wrapper class that accepts `n_starts`, `timeout_ms`, and `initial_guess_strategy` (warm, zero, random).
- Use a thread pool of worker threads created at startup (or lazily at first use). Each worker returns the first success it finds.
- Each worker receives per-thread reusable memory (via TLS) containing the `Robot` copy, `SQPIKSolver` instance (with pre-allocated arrays), and `RobotState` for FK.
- Cancellation: implement an atomic boolean `finished` shared among starts. When a worker converges, it writes the solution and sets `finished = true`, other workers check `if (finished) { cancel; return; }` between iterations.

### Thread Safety and Robot::clone
- Keep `Robot` immutable public data (kinematic graph and joint limits). Make cached states and solver objects owned outside `Robot`.
- Implement a `Robot::cloneForWorker()` that returns a lightweight copy where only runtime cached state is duplicated; or simply copy on creation if immutable internals are shared by shared_ptr.
- `SQPIKSolver` instances will be thread-local by design and include their own `DaQPSolver` wrapper.

### Eigen and Compiler Flags
- In non-MSVC compilers, add `-march=native` and `-O3` (if not set by CMAKE_BUILD_TYPE=Release) and define `EIGEN_NO_DEBUG` in the target to remove Eigen run-time checks and enable vectorization.
- Keep debug builds unchanged.

## Alternatives considered
- Single-thread multi-start: serially try many initial guesses until success — easier but slower. Rejected because the multistart racing approach can be faster and distributes compute across available cores.
- Dynamic memory reuse via custom allocator: considered but Eigen's internal allocation pattern is easier to manage via pre-sizing and .noalias(); custom allocator adds complexity.

## Risks & Mitigation
- Risk: Changing FK & Jacobian API breaks existing users. Mitigation: preserve previous compute() APIs and add overloads accepting `RobotState`, with default `compute(q)` calling new APIs internally.
- Risk: TLS & worker pool introduces complexity and lifetime management issues. Mitigation: Add tests and document the lifecycle; use RAII for pool management.

## Performance Validation
- Benchmarks should show: single-step solve microsecond level for 6-DOF; multistart N=4 within 1ms total time.
- Add microbench scripts under `benchmarks/` and integrate into CI for periodic profiling

## Migration Plan
1. Implement zero-allocation & DaQPSolver changes behind feature flag and tests.
2. Add `RobotState` and JacobianCalculator overload with tests.
3. Expose `RacingIKSolver` and TLS solver pool behind a switch and tests.
4. Update docs and add benchmarks.

## Open Questions
- How many starts should be default for racing (N)? Proposal defaults: N=4 with 1 warm start, 1 zero, N-2 random.
- Noise/randomness seed policy for deterministic bench runs.
- Windows/MSVC optimization flags: consult CI to ensure `-march=native` replacement on MSVC is appropriate (it doesn't support `-march`).