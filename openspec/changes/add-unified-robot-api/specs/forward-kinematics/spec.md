# forward-kinematics Specification Delta

## MODIFIED Requirements

### Requirement: System SHALL compute forward kinematics for kinematic chain using RobotModel
The ForwardKinematics class SHALL compute end-effector pose given joint angles using a RobotModel (previously called Robot) and Eigen transformations.

#### Scenario: Create ForwardKinematics with RobotModel
**GIVEN** a RobotModel parsed from UR5e URDF
**WHEN** the user creates `ForwardKinematics fk(model, "tool0")`
**THEN** the FK solver is initialized for the kinematic chain to "tool0"
**AND** the solver can compute transformations for any joint configuration

## ADDED Requirements

### Requirement: Robot class SHALL provide convenient forward kinematics methods
The Robot class SHALL provide simplified methods for computing forward kinematics without requiring users to instantiate ForwardKinematics objects.

#### Scenario: Compute FK using default end-effector
**GIVEN** a Robot instance created with `Robot::fromURDF("ur5e.urdf", "tool0")`
**AND** joint angles q = [0, -π/2, 0, -π/2, 0, 0]
**WHEN** the user calls `robot.forwardKinematics(q)`
**THEN** the system returns a Transform representing the pose of "tool0"
**AND** the result matches calling ForwardKinematics::compute() directly
**AND** no explicit ForwardKinematics object creation is needed

#### Scenario: Compute FK to specific link (override default)
**GIVEN** a Robot instance with default end-effector "tool0"
**AND** joint angles q = [0, 0, 0, 0, 0, 0]
**WHEN** the user calls `robot.forwardKinematics(q, "forearm_link")`
**THEN** the system returns the Transform to "forearm_link"
**AND** the default end-effector "tool0" is not changed for future calls
**AND** the Robot lazily creates a ForwardKinematics instance for "forearm_link"

#### Scenario: Compute FK with alternative method name
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.computePose(q)`
**THEN** the result is identical to `robot.forwardKinematics(q)`
**AND** both method names are supported for discoverability

#### Scenario: Repeated FK calls use cached solver
**GIVEN** a Robot instance
**WHEN** the user calls `robot.forwardKinematics(q1)` multiple times
**THEN** the internal ForwardKinematics object is created only once
**AND** subsequent calls reuse the cached instance
**AND** no performance penalty compared to direct ForwardKinematics usage

### Requirement: Robot class SHALL support cloning with independent FK state
The Robot class SHALL provide a clone() method that creates an independent copy suitable for multi-threaded usage.

#### Scenario: Clone robot and compute FK independently
**GIVEN** a Robot instance robot1 created from URDF
**WHEN** the user calls `auto robot2 = robot1.clone()`
**THEN** robot2 is a deep copy of robot1's RobotModel
**AND** robot2 has the same default end-effector and base link
**AND** robot2's FK computations are completely independent of robot1
**AND** both robots can be used concurrently from different threads

#### Scenario: Cloned robot does not share internal solvers
**GIVEN** robot1 has already computed FK (internal ForwardKinematics cached)
**WHEN** the user clones to create robot2
**THEN** robot2 starts with no cached solvers
**AND** robot2 lazily creates its own ForwardKinematics on first use
**AND** modifications to robot2's solvers do not affect robot1

#### Scenario: Clone is memory-independent
**GIVEN** a Robot instance robot1
**WHEN** robot1 is cloned to robot2
**AND** robot1 is destroyed/deleted
**THEN** robot2 continues to function normally
**AND** robot2's RobotModel is a separate memory allocation
