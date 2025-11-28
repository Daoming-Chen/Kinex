# jacobian-computation Specification

## Purpose
TBD - created by archiving change add-robotics-kinematics-library. Update Purpose after archive.
## Requirements
### Requirement: JacobianCalculator SHALL compute Jacobian matrices using RobotModel
The JacobianCalculator SHALL compute analytical and geometric Jacobians using a RobotModel (previously called Robot) and Eigen linear algebra.

#### Scenario: Create JacobianCalculator with RobotModel
**GIVEN** a RobotModel parsed from UR5e URDF
**WHEN** the user creates `JacobianCalculator calc(model, "tool0")`
**THEN** the calculator is initialized for the kinematic chain to "tool0"
**AND** the calculator can compute Jacobians for any joint configuration

### Requirement: System SHALL cache kinematic chain for efficient Jacobian evaluation
The system SHALL cache the kinematic chain structure for reuse across multiple Jacobian evaluations.

#### Scenario: Reuse cached chain for multiple Jacobian computations
**Given** a JacobianCalculator initialized with a robot model  
**When** the user calls compute(q) 100 times with different joint angles  
**Then** the kinematic chain is cached during initialization  
**And** subsequent calls use the cached structure  
**And** average computation time is < 0.5ms per call

### Requirement: System SHALL support both geometric and analytic Jacobians
The system SHALL compute both geometric (body) and analytic (spatial) Jacobians.

#### Scenario: Compute geometric Jacobian
**Given** a robot model  
**When** the user requests a geometric Jacobian  
**Then** the system returns J_geom that maps joint velocities to body twist  
**And** J_geom relates dq/dt to [v_linear, ω_angular] in end-effector frame

#### Scenario: Compute analytic Jacobian
**Given** a robot model  
**When** the user requests an analytic Jacobian  
**Then** the system returns J_analytic that maps joint velocities to spatial twist  
**And** J_analytic relates dq/dt to [v_linear, ω_angular] in base frame

### Requirement: System SHALL compute Jacobian to intermediate links
The system SHALL compute Jacobians for any link in the kinematic chain.

#### Scenario: Jacobian to intermediate link
**Given** a UR5e robot model  
**When** the user requests Jacobian to "forearm_link"  
**Then** the returned Jacobian has dimensions 6×n where n is the number of joints up to that link  
**And** the Jacobian excludes downstream joints

### Requirement: System SHALL handle singular configurations
The system SHALL detect and report singular configurations.

#### Scenario: Detect singularity
**Given** a robot in a singular configuration (e.g., UR5e elbow fully extended)  
**When** the user computes the Jacobian  
**Then** the system computes the Jacobian normally  
**And** provides a method to check singularity via `isSingular()`  
**And** `isSingular()` returns true if smallest singular value < threshold

#### Scenario: Query manipulability measure
**Given** a computed Jacobian matrix J  
**When** the user calls `getManipulability()`  
**Then** the system returns sqrt(det(J*J^T))  
**And** the value is close to 0 near singularities

### Requirement: System SHALL provide Jacobian derivatives (Hessian)
The system SHALL optionally compute second-order derivatives for acceleration analysis.

#### Scenario: Compute Jacobian time derivative
**Given** a robot model, joint angles q, and joint velocities dq  
**When** the user calls `computeJacobianDerivative(q, dq)`  
**Then** the system returns dJ/dt using analytical methods  
**And** the result is used for acceleration-level IK

### Requirement: System SHALL efficient memory management
The system SHALL minimize dynamic allocation during Jacobian computation.

#### Scenario: Zero allocation after initialization
**Given** a JacobianCalculator initialized once  
**When** compute(q) is called repeatedly  
**Then** no heap allocations occur during compute()  
**And** all matrices are pre-allocated and reused

### Requirement: System SHALL ensure numerical stability
The system SHALL use numerically stable algorithms for Jacobian computation.

#### Scenario: Jacobian for small joint angles
**Given** a robot with very small joint angles (< 1e-10 rad)  
**When** the Jacobian is computed  
**Then** the result has no NaN or Inf values  
**And** the relative error compared to analytical solution is < 1e-6

### Requirement: Robot class SHALL provide convenient Jacobian computation methods
The Robot class SHALL provide simplified methods for computing Jacobians and manipulability metrics without requiring users to instantiate JacobianCalculator objects.

#### Scenario: Compute Jacobian using default end-effector
**GIVEN** a Robot instance created with `Robot::fromURDF("ur5e.urdf", "tool0")`
**AND** joint angles q = [0, -π/2, 0, -π/2, 0, 0]
**WHEN** the user calls `robot.computeJacobian(q)`
**THEN** the system returns a 6×N Jacobian matrix (N = number of joints)
**AND** the result matches calling JacobianCalculator::compute() directly
**AND** the Jacobian type is Analytic by default

#### Scenario: Compute Jacobian for specific link
**GIVEN** a Robot instance with default end-effector "tool0"
**AND** joint angles q
**WHEN** the user calls `robot.computeJacobian(q, "wrist_2_link")`
**THEN** the system returns the Jacobian for "wrist_2_link"
**AND** the default end-effector is not changed
**AND** the Robot lazily creates a JacobianCalculator for "wrist_2_link"

#### Scenario: Compute geometric Jacobian
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.computeJacobian(q, "", JacobianType::Geometric)`
**THEN** the system returns the geometric (body) Jacobian
**AND** the Jacobian is expressed in the end-effector frame

#### Scenario: Repeated Jacobian calls use cached calculator
**GIVEN** a Robot instance
**WHEN** the user calls `robot.computeJacobian(q1)` then `robot.computeJacobian(q2)`
**THEN** the internal JacobianCalculator is created only once
**AND** both calls reuse the cached instance
**AND** no performance penalty compared to direct usage

### Requirement: Robot class SHALL provide manipulability analysis methods
The Robot class SHALL expose manipulability metrics and singularity detection for configuration analysis.

#### Scenario: Compute Yoshikawa manipulability measure
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.getManipulability(q)`
**THEN** the system returns the Yoshikawa manipulability index
**AND** the value is non-negative
**AND** the value approaches zero near singularities
**AND** high values indicate better manipulability

#### Scenario: Check for singular configuration
**GIVEN** a Robot instance
**AND** joint angles q placing the robot near a singularity
**WHEN** the user calls `robot.isSingular(q)`
**THEN** the system returns true if the Jacobian is singular (default threshold 1e-6)
**AND** the user can override threshold: `robot.isSingular(q, 1e-4)`

#### Scenario: Get Jacobian condition number
**GIVEN** a Robot instance
**AND** joint angles q
**WHEN** the user calls `robot.getConditionNumber(q)`
**THEN** the system returns the condition number of the Jacobian
**AND** large values (>100) indicate ill-conditioned configurations
**AND** values near 1.0 indicate well-conditioned configurations

#### Scenario: Manipulability methods use default end-effector
**GIVEN** a Robot with default end-effector "tool0"
**WHEN** the user calls manipulability methods without link parameter
**THEN** all methods analyze manipulability for "tool0"
**AND** results are consistent across all methods

#### Scenario: Manipulability methods accept custom link
**GIVEN** a Robot instance
**WHEN** the user calls `robot.getManipulability(q, "wrist_3_link")`
**THEN** the system analyzes manipulability for "wrist_3_link"
**AND** the JacobianCalculator for "wrist_3_link" is cached for reuse

