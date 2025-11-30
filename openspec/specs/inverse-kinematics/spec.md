# inverse-kinematics Specification

## Purpose
TBD - created by archiving change add-robotics-kinematics-library. Update Purpose after archive.
## Requirements
### Requirement: IKSolver SHALL solve inverse kinematics using RobotModel
The SQPIKSolver SHALL compute joint angles to achieve a target end-effector pose using a RobotModel (previously called Robot) and SQP optimization.

#### Scenario: Create IKSolver with RobotModel
**GIVEN** a RobotModel parsed from UR5e URDF
**WHEN** the user creates `SQPIKSolver solver(model, "tool0")`
**THEN** the IK solver is initialized for the kinematic chain to "tool0"
**AND** the solver can solve IK for any valid target pose

### Requirement: System SHALL respect joint limits as constraints
The system SHALL enforce joint limits as hard constraints during IK solving.

#### Scenario: IK solution respects joint limits
**Given** a UR5e robot with joint limits [-π, π]  
**And** a target pose that would require joint angles exceeding limits without constraints  
**When** IK is solved with joint limit enforcement  
**Then** the solution has all joint angles within [-π, π]  
**And** the solution is the best reachable pose within limits

#### Scenario: Unreachable pose due to joint limits
**Given** a target pose only reachable with joints outside limits  
**When** IK is solved  
**Then** the solver returns failure status after max iterations  
**And** the best effort solution is returned  
**And** the error residual is reported

### Requirement: System SHALL handle multiple solution candidates
The system SHALL support providing different initial guesses to find alternative IK solutions.

#### Scenario: Find elbow-up vs elbow-down solutions
**Given** a UR5e robot and a target pose with multiple IK solutions  
**When** the user solves IK with different initial guesses (elbow-up and elbow-down)  
**Then** the solver converges to different valid solutions  
**And** both solutions satisfy FK(q) �?target

### Requirement: System SHALL configure solver parameters
The system SHALL allow configuration of solver parameters for different use cases.

#### Scenario: Set convergence tolerance
**Given** an IKSolver instance  
**When** the user sets position tolerance to 0.001m and orientation tolerance to 0.001 rad  
**Then** the solver only returns success if errors are below these thresholds

#### Scenario: Set maximum iterations
**Given** an IKSolver instance  
**When** the user sets max iterations to 50  
**Then** the solver terminates after 50 iterations if not converged  
**And** returns the best solution found

#### Scenario: Configure step size limits
**Given** an IKSolver instance  
**When** the user sets max joint step to 0.1 rad  
**Then** each SQP iteration limits Δq to 0.1 rad per joint  
**And** this prevents large jumps for numerical stability

### Requirement: System SHALL report solver diagnostics
The system SHALL provide detailed diagnostics about the solving process.

#### Scenario: Query solver status after solving
**Given** a completed IK solve  
**When** the user queries the solver status  
**Then** the status includes convergence flag, iterations taken, final error, and constraint violations  
**And** the status indicates if max iterations was reached

#### Scenario: Access convergence history
**Given** an IK solve with history logging enabled  
**When** the user queries the convergence history  
**Then** the system returns error norms and joint angles for each iteration  
**And** this data can be used for debugging or visualization  
**And** iteration details are logged using spdlog at DEBUG level

### Requirement: System SHALL handle unreachable poses gracefully
The system SHALL handle targets outside the workspace without crashing.

#### Scenario: Target beyond reach
**Given** a target pose 10 meters away from robot base  
**When** IK is solved  
**Then** the solver returns failure status  
**And** the best effort solution points toward the target  
**And** no exceptions are thrown

### Requirement: System SHALL support position-only and orientation-only IK
The system SHALL allow solving IK for position or orientation independently.

#### Scenario: Solve IK for position only
**Given** a target position without orientation constraint  
**When** the user calls `ikSolver.solvePosition(targetPosition)`  
**Then** the solver finds q such that FK(q).position �?targetPosition  
**And** the orientation is free (not constrained)

#### Scenario: Solve IK for orientation only
**Given** a target orientation without position constraint  
**When** the user calls `ikSolver.solveOrientation(targetOrientation)`  
**Then** the solver finds q such that FK(q).orientation �?targetOrientation  
**And** the position is free (not constrained)

### Requirement: System SHALL optimize for smoothness in trajectory IK
The system SHALL minimize joint velocity when solving sequential IK problems.

#### Scenario: Trajectory IK with smoothness penalty
**Given** a sequence of target poses  
**When** IK is solved with a smoothness weight parameter  
**Then** the joint trajectory minimizes ||q_i - q_{i-1}||  
**And** the trajectory has no unnecessary oscillations

### Requirement: System SHALL provide thread-safe IK solving
The system SHALL support concurrent IK solving from multiple threads.

#### Scenario: Parallel IK solving
**Given** multiple IKSolver instances (one per thread)  
**When** 10 threads solve IK simultaneously for different targets  
**Then** all solves complete successfully  
**And** results are independent and correct  
**And** no data races occur

### Requirement: Robot class SHALL provide convenient inverse kinematics methods
The Robot class SHALL provide simplified methods for solving IK without requiring users to instantiate IKSolver objects.

#### Scenario: Solve IK using default end-effector
**GIVEN** a Robot instance created with `Robot::fromURDF("ur5e.urdf", "tool0")`
**AND** a target pose T_target
**AND** an initial guess q_init = [0, 0, 0, 0, 0, 0]
**WHEN** the user calls `auto [q_solution, status] = robot.inverseKinematics(T_target, q_init)`
**THEN** the system returns joint angles q_solution
**AND** status indicates convergence success/failure
**AND** the result matches calling SQPIKSolver::solve() directly

#### Scenario: Solve IK for specific link (override default)
**GIVEN** a Robot instance with default end-effector "tool0"
**AND** a target pose for "wrist_3_link"
**WHEN** the user calls `robot.inverseKinematics(T_target, q_init, "wrist_3_link")`
**THEN** the system solves IK to position "wrist_3_link" at T_target
**AND** the default end-effector is not changed
**AND** the Robot lazily creates an IKSolver for "wrist_3_link"

#### Scenario: Solve IK with alternative method name
**GIVEN** a Robot instance
**AND** a target pose T_target
**WHEN** the user calls `robot.solveIK(T_target, q_init)`
**THEN** the result is identical to `robot.inverseKinematics(T_target, q_init)`
**AND** both method names are supported

#### Scenario: IK solver configuration persists across calls
**GIVEN** a Robot instance
**WHEN** the user sets `robot.setIKTolerance(1e-6)`
**AND** calls `robot.inverseKinematics(T1, q_init1)`
**AND** later calls `robot.inverseKinematics(T2, q_init2)`
**THEN** both IK calls use tolerance 1e-6
**AND** the configuration persists in the cached solver

### Requirement: Robot class SHALL support IK solver configuration
The Robot class SHALL allow users to configure IK solver parameters through the unified API.

#### Scenario: Configure IK tolerance
**GIVEN** a Robot instance
**WHEN** the user calls `robot.setIKTolerance(1e-6)`
**THEN** subsequent IK calls use tolerance 1e-6
**AND** the solver converges with higher precision

#### Scenario: Enable position-only IK
**GIVEN** a Robot instance
**WHEN** the user calls `robot.setPositionOnlyIK(true)`
**THEN** subsequent IK calls ignore orientation constraints
**AND** only position error is minimized
**AND** convergence is faster for position-only problems

#### Scenario: Set full solver configuration
**GIVEN** a Robot instance
**AND** a custom SolverConfig with max_iterations=200, tolerance=1e-5
**WHEN** the user calls `robot.setSolverConfig(config)`
**THEN** the IK solver uses all parameters from config
**AND** the config can be retrieved via `robot.getSolverConfig()`

#### Scenario: Cloned robots have independent IK configurations
**GIVEN** robot1 with tolerance set to 1e-6
**WHEN** robot1 is cloned to robot2
**AND** robot2.setIKTolerance(1e-4) is called
**THEN** robot1 still uses tolerance 1e-6
**AND** robot2 uses tolerance 1e-4
**AND** their IK solvers are completely independent

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

