# inverse-kinematics Specification Delta

## MODIFIED Requirements

### Requirement: IKSolver SHALL solve inverse kinematics using RobotModel
The SQPIKSolver SHALL compute joint angles to achieve a target end-effector pose using a RobotModel (previously called Robot) and SQP optimization.

#### Scenario: Create IKSolver with RobotModel
**GIVEN** a RobotModel parsed from UR5e URDF
**WHEN** the user creates `SQPIKSolver solver(model, "tool0")`
**THEN** the IK solver is initialized for the kinematic chain to "tool0"
**AND** the solver can solve IK for any valid target pose

## ADDED Requirements

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
