# wasm-bindings Specification

## Purpose
TBD - created by archiving change add-robotics-kinematics-library. Update Purpose after archive.
## Requirements
### Requirement: System SHALL compile library to WebAssembly using Emscripten
The system SHALL compile the C++ library to WebAssembly for browser usage.

#### Scenario: Build WASM module with Emscripten
**Given** the kinex C++ library  
**When** the user runs the WASM build script  
**Then** a kinex.js and kinex.wasm file are generated  
**And** the combined size is < 2MB (uncompressed)  
**And** the module works in modern browsers

#### Scenario: Load WASM module in browser
**Given** the built kinex WASM module  
**When** a web page loads the module with `import kinex from './kinex.js'`  
**Then** the module loads successfully  
**And** all exported functions are accessible  
**And** no errors appear in browser console

### Requirement: WASM bindings SHALL use RobotModel internally
The WASM bindings SHALL update RobotHandle to use RobotModel (previously Robot) internally while maintaining existing JavaScript API.

#### Scenario: RobotHandle wraps RobotModel
**GIVEN** the WASM bindings implementation
**WHEN** a user creates a Robot from JavaScript
**THEN** internally RobotHandle holds a `std::shared_ptr<kinex::RobotModel>`
**AND** the JavaScript API remains unchanged for backward compatibility

### Requirement: System SHALL provide TypeScript type definitions
The system SHALL include TypeScript declaration files (.d.ts) for type safety.

#### Scenario: TypeScript type checking
**Given** TypeScript code using kinex WASM module  
**When** the user compiles with tsc  
**Then** all kinex types are recognized  
**And** type errors are caught at compile time  
**And** IDE provides accurate autocomplete

### Requirement: System SHALL optimize WASM binary size
The system SHALL minimize WebAssembly binary size through optimization.

#### Scenario: WASM binary size under limit
**Given** the compiled kinex.wasm file  
**When** measured after compression (gzip)  
**Then** the size is < 500KB  
**And** the module loads in < 1 second on a typical connection

#### Scenario: Tree-shake unused code
**Given** WASM build configuration  
**When** only FK is used (not IK)  
**Then** the IK solver code is eliminated from the binary  
**And** the binary size is smaller than the full build

### Requirement: System SHALL enable SIMD for performance
The system SHALL use WebAssembly SIMD instructions for Eigen operations.

#### Scenario: SIMD-optimized matrix operations
**Given** a WASM build with SIMD enabled  
**When** FK is computed with matrix multiplications  
**Then** SIMD instructions are used (verify with WASM inspector)  
**And** performance is 2-3× faster than scalar version

### Requirement: System SHALL handle memory management safely
The system SHALL properly manage memory allocated in WASM heap.

#### Scenario: No memory leaks in repeated FK calls
**Given** a loaded WASM module  
**When** FK is computed 10,000 times in a loop  
**Then** WASM heap usage remains constant  
**And** no memory leaks are detected

#### Scenario: Explicit resource cleanup
**Given** a Robot object created in JavaScript  
**When** the user calls `robot.delete()`  
**Then** the C++ object is destroyed  
**And** memory is freed in the WASM heap

### Requirement: System SHALL support file system access via Emscripten FS
The system SHALL support loading URDF files from virtual filesystem.

#### Scenario: Load URDF from virtual FS
**Given** a URDF file preloaded into Emscripten's virtual filesystem  
**When** the user calls `Robot.fromURDF("/models/ur5e.urdf")`  
**Then** the file is read from virtual FS  
**And** the robot is parsed correctly

#### Scenario: Fetch URDF from URL
**Given** a URDF file at a remote URL  
**When** the user fetches the content and creates robot from string  
**Then** the WASM module parses it successfully  
**And** the robot model is usable

### Requirement: System SHALL provide JavaScript-friendly API
The system SHALL adapt C++ API to JavaScript idioms.

#### Scenario: Return plain JavaScript objects
**Given** a FK computation result  
**When** returned to JavaScript  
**Then** the pose is a plain JS object `{position: [x,y,z], rotation: [qw,qx,qy,qz]}`  
**And** no C++ proxy objects are returned (for serialization)

#### Scenario: Accept JavaScript arrays for vectors
**Given** a function expecting joint angles  
**When** the user passes a JavaScript array [0, 1, 2, 3, 4, 5]  
**Then** the function accepts it without conversion  
**And** the array is automatically converted to std::vector

### Requirement: System SHALL support promise-based async operations
The system SHALL support asynchronous operations for expensive computations.

#### Scenario: Async IK solving
**Given** an IK problem that may take 100ms  
**When** the user calls `await ikSolver.solveAsync(target)`  
**Then** the computation runs without blocking the main thread  
**And** the result is returned via Promise

### Requirement: System SHALL be compatible with modern JavaScript bundlers
The system SHALL work with Webpack, Vite, and other bundlers.

#### Scenario: Import in Vite application
**Given** a Vite React project  
**When** the user imports kinex with `import kinex from 'kinex'`  
**Then** Vite bundles the WASM module correctly  
**And** the WASM file is copied to output directory  
**And** the module loads at runtime

### Requirement: System SHALL provide usage examples and documentation
The system SHALL include JavaScript/TypeScript examples.

#### Scenario: Access example code
**Given** the kinex repository  
**When** the user browses the examples/ directory  
**Then** JavaScript examples are available for FK, IK, and Jacobian  
**And** each example includes clear comments  
**And** examples can run in browser or Node.js

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

