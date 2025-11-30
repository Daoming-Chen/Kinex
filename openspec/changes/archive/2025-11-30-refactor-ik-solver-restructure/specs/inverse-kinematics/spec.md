## ADDED Requirements

### Requirement: IKSolver SHALL perform zero-allocation hot-path inner loops
The SQPIKSolver implementation SHALL reuse preallocated member matrices and vectors for iterative solves so that no heap allocations occur within the SQP iteration loop.

#### Scenario: Repeated solves allocate no heap memory
**GIVEN** a SQPIKSolver initialized for a 6-DOF robot
**WHEN** the user calls `solver.solve(target, q_initial)` 1000 times
**THEN** the hot path (solver iterations) must not perform heap allocations
**AND** each per-solve average time is < 200 microseconds on the benchmark hardware

### Requirement: IKSolver SHALL accept RobotState for FK/Jacobian fusion
The SQPIKSolver and related APIs SHALL allow providing a `RobotState` (precomputed transforms) to `solve()` so that FK transforms are not recomputed during Jacobian computations, enabling reuse across Jacobian and FK calls.

#### Scenario: Solve with provided RobotState
**GIVEN** a precomputed `RobotState state` produced by ForwardKinematics
**AND** a SQPIKSolver instance
**WHEN** the user calls `solver.solve(target, q_init, robot_state=state)`
**THEN** the solver uses the transforms in `state` to compute errors and Jacobians
**AND** it does not recompute link transforms internally

### Requirement: IKSolver SHALL support a racing multi-start strategy
The SQPIKSolver or a wrapper `RacingIKSolver` SHALL optionally run multiple independent IK attempts with different initial guesses concurrently and return the first successful result.

#### Scenario: Racing with multiple starts returns first success
**GIVEN** a `RacingIKSolver` configured with 4 starts including warm start and random seeds
**WHEN** racing is started for a target pose
**THEN** the `RacingIKSolver` returns when the first worker converges
**AND** the total runtime is less than or equal to the configured time budget
**AND** other workers either stop computing or are canceled

### Requirement: IKSolver SHALL provide a configurable solver pool and thread-safety
The system SHALL expose configuration to enable TLS solver pool usage. Each worker thread in the pool SHALL own independent `Robot` and `SQPIKSolver` instances, and the solver pool SHALL avoid `Robot::clone()` overhead during hot solves.

#### Scenario: Multi-threaded racing with TLS solver pool
**GIVEN** a `RacingIKSolver` configured to use a worker pool of 4 threads
**WHEN** 4 threads are concurrently solving IK (different targets)
**THEN** each solve uses its thread-local `SQPIKSolver` and `Robot` instance
**AND** no data races occur
**AND** the racing strategy produces identical output to sequential solves within tolerance

## ADDED Requirements

### Requirement: DaQPSolver wrapper SHALL persist QP arrays
The `DaQPSolver` wrapper SHALL store `lower_`, `upper_`, and `x_` arrays as persistent members and expose zero-copy pointers to the underlying DAQP C API to prevent vector copies.

#### Scenario: QP solve with persistent arrays
**GIVEN** a `DaQPSolver` wrapper instance initialized for `n` DOF
**WHEN** a `solve` call occurs many times in sequence
**THEN** no heap allocations occur due to per-call vector copies
**AND** the QP arrays are resized during initialization only

### Requirement: IKSolver SHALL expose warm-start and configurable initial guess policies
The `IKSolver` interface SHALL support warm-start (previous frame's q), zero initial guess, and random within joint limits as initial guess policies for multi-start strategies.

#### Scenario: Warm start used for a start slot
**GIVEN** a `RacingIKSolver` configured with `startStrategies = [WarmStart, Zero, Random, Random]`
**WHEN** a new solve is performed with a previous q solution available
**THEN** one start uses the previous q as initial guess and may converge faster than other starts

### Requirement: IKSolver SHALL expose diagnostics on multi-start runs
The `IKSolver` SHALL expose the results and diagnostic summary for all attempted starts even if the first converging start wins.

#### Scenario: Query full multi-start diagnostics
**GIVEN** a `RacingIKSolver` that attempted 4 starts
**WHEN** the user queries the solve diagnostics
**THEN** the system returns per-start convergence flags, iterations, final error, and time spent

### Requirement: IKSolver SHALL provide escape mechanisms for local minima
The multi-start racing solver SHALL be the default method to avoid local minima; for single-start solves, the solver SHALL expose a retry policy that attempts additional starts whenever single-start fails under a given time budget.

#### Scenario: Retry policy after single start fails
**GIVEN** a single-start `SQPIKSolver` configured with `enable_retry=true` and `timeout_ms=1` and `max_retries=3`
**WHEN** the initial attempt fails to converge
**THEN** the solver tries additional initial guesses up to `max_retries` or the `timeout_ms` budget is reached
**AND** each retry shares no cross-call memory allocation
