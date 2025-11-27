# wasm-bindings Specification Delta

## MODIFIED Requirements

### Requirement: WASM bindings SHALL use RobotModel internally
The WASM bindings SHALL update RobotHandle to use RobotModel (previously Robot) internally while maintaining existing JavaScript API.

#### Scenario: RobotHandle wraps RobotModel
**GIVEN** the WASM bindings implementation
**WHEN** a user creates a Robot from JavaScript
**THEN** internally RobotHandle holds a `std::shared_ptr<kinex::RobotModel>`
**AND** the JavaScript API remains unchanged for backward compatibility

## ADDED Requirements

### Requirement: WASM bindings SHALL expose unified Robot class
The WASM bindings SHALL provide a JavaScript-accessible Robot class that wraps the C++ unified Robot API using Embind.

#### Scenario: Create Robot from URDF string in JavaScript
**GIVEN** a URDF XML string loaded in the browser
**WHEN** the user executes `const robot = kinex.Robot.fromURDFString(urdfContent, "tool0")`
**THEN** a Robot instance is created in the WASM environment
**AND** the instance is usable from JavaScript
**AND** the default end-effector is "tool0"

#### Scenario: Compute FK from JavaScript
**GIVEN** a JavaScript Robot instance
**AND** joint angles as JavaScript array [0, -1.57, 0, -1.57, 0, 0]
**WHEN** the user calls `const pose = robot.forwardKinematics(jointAngles)`
**THEN** the system returns a Pose object with {position: Vec3, quaternion: Quat}
**AND** pose.asMatrix() returns a Matrix object with Float64Array data

#### Scenario: Compute FK with alternative method name
**GIVEN** a JavaScript Robot instance
**WHEN** the user calls `const pose = robot.computePose(jointAngles)`
**THEN** the result is identical to `robot.forwardKinematics(jointAngles)`

#### Scenario: Solve IK from JavaScript
**GIVEN** a JavaScript Robot instance
**AND** a target pose as {position: [x,y,z], quaternion: [w,x,y,z]}
**AND** an initial guess as array of joint angles
**WHEN** the user calls `const result = robot.inverseKinematics(targetPose, initialGuess)`
**THEN** result.solution is a Float64Array of joint angles
**AND** result.status contains {converged: boolean, iterations: number, error: number, ...}
**AND** the operation runs efficiently in WASM

#### Scenario: Solve IK with alternative method name
**GIVEN** a JavaScript Robot instance
**WHEN** the user calls `const result = robot.solveIK(target, guess)`
**THEN** the result is identical to `robot.inverseKinematics(target, guess)`

#### Scenario: Compute Jacobian from JavaScript
**GIVEN** a JavaScript Robot instance
**AND** joint angles as array
**WHEN** the user calls `const J = robot.computeJacobian(jointAngles)`
**THEN** the system returns a Matrix object
**AND** J.data is a Float64Array of length 6 × n_joints (row-major order)
**AND** J.rows === 6 and J.cols === n_joints

#### Scenario: Get manipulability metrics from JavaScript
**GIVEN** a JavaScript Robot instance
**AND** joint angles as array
**WHEN** the user calls `const m = robot.getManipulability(jointAngles)`
**THEN** m is a JavaScript number (manipulability index)
**WHEN** the user calls `const isSing = robot.isSingular(jointAngles)`
**THEN** isSing is a JavaScript boolean
**WHEN** the user calls `const cond = robot.getConditionNumber(jointAngles)`
**THEN** cond is a JavaScript number

#### Scenario: Configure IK solver from JavaScript
**GIVEN** a JavaScript Robot instance
**WHEN** the user calls `robot.setIKTolerance(1e-6)`
**THEN** subsequent IK calls use tolerance 1e-6
**WHEN** the user calls `robot.setPositionOnlyIK(true)`
**THEN** IK ignores orientation
**WHEN** the user calls `const config = robot.getSolverConfig()`
**THEN** config is a SolverConfig JavaScript object

#### Scenario: Clone Robot in WASM
**GIVEN** a JavaScript Robot instance robot1
**WHEN** the user calls `const robot2 = robot1.clone()`
**THEN** robot2 is an independent copy in WASM memory
**AND** both instances can coexist and operate independently
**AND** memory management is handled by WASM/Embind

#### Scenario: Robot properties accessible from JavaScript
**GIVEN** a JavaScript Robot instance
**THEN** `robot.getName()` returns the robot name string
**AND** `robot.getDOF()` returns degrees of freedom
**AND** `robot.getEndLink()` returns end-effector link name
**AND** `robot.getBaseLink()` returns base link name
**AND** `robot.getJointNames()` returns array of joint name strings

#### Scenario: Memory cleanup for Robot
**GIVEN** a JavaScript Robot instance
**WHEN** the instance goes out of scope
**THEN** the WASM/Embind smart pointer management automatically frees memory
**AND** no manual dispose() call is required for Robot (unlike RobotHandle)

### Requirement: WASM bindings SHALL maintain backward compatibility with RobotHandle
The WASM bindings SHALL keep the existing RobotHandle class working for users who depend on the current API.

#### Scenario: RobotHandle API continues to work
**GIVEN** existing JavaScript code using RobotHandle
**WHEN** the code creates `const robot = kinex.Robot.fromURDFString(urdf)`
**AND** uses ForwardKinematicsHandle, JacobianCalculatorHandle, SQPIKSolverHandle
**THEN** all existing APIs continue to function
**AND** RobotHandle internally uses RobotModel instead of Robot
**AND** no breaking changes to existing users
