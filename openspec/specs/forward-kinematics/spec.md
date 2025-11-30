# forward-kinematics Specification

## Purpose
TBD - created by archiving change add-robotics-kinematics-library. Update Purpose after archive.
## Requirements
### Requirement: System SHALL compute forward kinematics for kinematic chain using RobotModel
The ForwardKinematics class SHALL compute end-effector pose given joint angles using a RobotModel (previously called Robot) and Eigen transformations.

#### Scenario: Create ForwardKinematics with RobotModel
**GIVEN** a RobotModel parsed from UR5e URDF
**WHEN** the user creates `ForwardKinematics fk(model, "tool0")`
**THEN** the FK solver is initialized for the kinematic chain to "tool0"
**AND** the solver can compute transformations for any joint configuration

### Requirement: System SHALL support multiple end-effectors
The system SHALL compute FK for any link in the kinematic chain.

#### Scenario: Compute FK to intermediate link
**Given** a UR5e robot model  
**When** the user requests FK to link "forearm_link" instead of tool  
**Then** the system returns the transformation from base to forearm_link  
**And** the result excludes downstream joints

### Requirement: System SHALL provide efficient FK computation
The system SHALL compute FK efficiently by caching static transformations.

#### Scenario: Multiple FK calls with same robot
**Given** a ForwardKinematics object initialized with a robot  
**When** the user calls compute() 1000 times with different joint angles  
**Then** the average computation time is < 1ms per call  
**And** no dynamic memory allocation occurs during compute()

### Requirement: System SHALL return pose in multiple formats
The system SHALL provide FK results in various transformation representations.

#### Scenario: Get FK as 4x4 matrix
**Given** a computed FK result  
**When** the user calls `result.asMatrix()`  
**Then** the system returns an Eigen::Matrix4d homogeneous transformation

#### Scenario: Get FK as position and quaternion
**Given** a computed FK result  
**When** the user calls `result.asPositionQuaternion()`  
**Then** the system returns a std::pair<Eigen::Vector3d, Eigen::Quaterniond>  
**And** the quaternion is normalized

#### Scenario: Get FK as position and Euler angles
**Given** a computed FK result  
**When** the user calls `result.asPositionEuler()`  
**Then** the system returns position and roll-pitch-yaw Euler angles  
**And** the angles are in radians

### Requirement: System SHALL validate joint angle bounds
The system SHALL optionally validate joint angles against limits during FK computation.

#### Scenario: FK with joints within limits
**Given** a robot with joint limits  
**And** joint angles within the limits  
**When** FK is computed with bounds checking enabled  
**Then** the computation succeeds normally

#### Scenario: FK with joints exceeding limits
**Given** a robot with joint limits [-π, π]  
**And** joint angles including a value of 2π  
**When** FK is computed with strict bounds checking  
**Then** the system throws a `JointLimitException`  
**And** the exception identifies which joint violated limits

#### Scenario: FK without bounds checking
**Given** joint angles exceeding limits  
**When** FK is computed with bounds checking disabled  
**Then** the computation proceeds and returns a result  
**And** no exception is thrown

### Requirement: System SHALL provide thread-safe FK computation
The system SHALL allow concurrent FK computations on the same robot model.

#### Scenario: Parallel FK calls from multiple threads
**Given** a single ForwardKinematics object  
**When** 10 threads call compute() simultaneously with different joint angles  
**Then** all calls complete successfully  
**And** each result corresponds to its input joint angles  
**And** no race conditions occur

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

### Requirement: ForwardKinematics SHALL provide `RobotState` and compute FK efficiently
The ForwardKinematics class SHALL provide a `computeRobotState(q, RobotState&)` API that computes the per-link transforms and pre-allocates all necessary transforms so FK/Jacobian fusion is possible. ForwardKinematics SHALL avoid heap allocations during repeated calls.

#### Scenario: Provide RobotState via FK
**GIVEN** a `ForwardKinematics` instance
**WHEN** the user calls `computeRobotState(q, state)`
**THEN** `state` is populated with per-link transforms and `compute` returns with no heap allocation in the hot path

### Requirement: FK and Jacobian fusion via `computeFKAndJacobian` overload
The ForwardKinematics or a combined helper API SHALL provide a `computeFKAndJacobian(q, RobotState&, J)` that returns both the end-effector pose and Jacobian using shared transforms without recomputation.

#### Scenario: Combined FK & Jacobian returns same values as separate calls
**GIVEN** a `ForwardKinematics` instance and `JacobianCalculator` instance
**WHEN** the user calls `computeFKAndJacobian(q, state, J)`, collecting both pose and Jacobian
**THEN** results match `compute(q)` and `JacobianCalculator::compute(q)` within numerical tolerance

### Requirement: FK SHALL allow `Robot::cloneForWorker` and fast per-thread state
The `Robot` class SHALL expose a `cloneForWorker()` or factory method suitable for TLS copying so that `ForwardKinematics` objects using the clone have fast, independent per-thread runtime state and minimal memory overhead.

#### Scenario: Clone for worker duplicates runtime state only
**GIVEN** a `Robot model` and `ForwardKinematics fk`
**WHEN** the user calls `auto worker_robot = robot.cloneForWorker()`
**THEN** `worker_robot` shares immutable model internals and contains independent FK and Jacobian caches
**AND** `worker_robot` uses less memory than a full deep clone

