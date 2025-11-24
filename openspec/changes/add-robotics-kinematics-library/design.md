# Design: Robotics Kinematics Library

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────�?
�?                    kinex Library Core                      �?
�? ┌──────────────�? ┌──────────────�? ┌──────────────�?    �?
�? �?URDF Parser  │→ �?  Forward    │→ �?  Jacobian   �?    �?
�? �? (pugixml)   �? �? Kinematics  �? │Computation   �?    �?
�? └──────────────�? �?  (Eigen)    �? �?(Analytical) �?    �?
�?                    └──────────────�? └──────────────�?    �?
�?                           �?                 �?            �?
�?                    ┌──────────────────────────────�?      �?
�?                    �?  Inverse Kinematics         �?      �?
�?                    �?  (DaQP + SQP)               �?      �?
�?                    └──────────────────────────────�?      �?
└─────────────────────────────────────────────────────────────�?
                �?                             �?
    ┌──────────────────────�?     ┌──────────────────────�?
    �? Python Bindings     �?     �? WASM Bindings       �?
    �?   (nanobind)        �?     �?  (Emscripten)       �?
    └──────────────────────�?     └──────────────────────�?
                                              �?
                                  ┌──────────────────────�?
                                  �? Visualization App   �?
                                  �?    (Three.js)       �?
                                  └──────────────────────�?
```

## Component Design

### 1. URDF Parser
**Responsibility**: Parse URDF XML into in-memory robot model

**Key Classes**:
- `Robot`: Root structure containing links and joints
- `Link`: Represents robot link with inertial, visual, collision properties
- `Joint`: Represents joint with type, axis, limits, dynamics
- `URDFParser`: Parses XML using pugixml

**Design Decisions**:
- Use pugixml for lightweight XML parsing (no external dependencies)
- Store robot structure as directed acyclic graph (DAG)
- Cache kinematic chain for efficient FK/IK computation

### 2. Forward Kinematics
**Responsibility**: Compute end-effector pose from joint angles

**Key Classes**:
- `KinematicChain`: Represents chain from base to end-effector
- `ForwardKinematics`: Computes FK using DH parameters or direct transforms
- `Transform`: Wrapper around Eigen::Isometry3d

**Design Decisions**:
- Use Eigen for all matrix operations
- Support both DH parameters and direct transformation approach
- Pre-compute static transformations for efficiency

**Algorithm**:
```cpp
T_end = T_base;
for (joint in chain) {
    T_end *= joint.getTransform(q[i]);
}
return T_end;
```

### 3. Jacobian Computation
**Responsibility**: Compute Jacobian matrix using analytical geometric methods

**Key Classes**:
- `JacobianCalculator`: Uses analytical geometric methods to compute Jacobian
- Geometric formulas based on kinematic structure

**Design Decisions**:
- Use closed-form geometric formulas for revolute and prismatic joints
- Cache kinematic chain structure for efficient repeated computation
- Support both geometric and analytic Jacobians

**Geometric Method**:
```cpp
// For revolute joint i:
// J_linear[i] = z_i × (p_ee - p_i)
// J_angular[i] = z_i

// For prismatic joint i:
// J_linear[i] = z_i
// J_angular[i] = 0

// Compute Jacobian
Eigen::MatrixXd J = calculator.compute(q);
```

### 4. Inverse Kinematics
**Responsibility**: Solve for joint angles given target pose

**Key Classes**:
- `IKSolver`: Abstract base class
- `SQPIKSolver`: SQP-based solver using DaQP
- `IKProblem`: Defines objective and constraints

**Design Decisions**:
- Use Sequential Quadratic Programming (SQP) approach
- Formulate as optimization problem:
  - Minimize: ||FK(q) - target||²
  - Subject to: q_min �?q �?q_max
- Use DaQP for efficient QP solving
- Support warm-starting for trajectory generation

**SQP Algorithm**:
```
1. Initialize q₀
2. While not converged:
   a. Compute J = Jacobian(q)
   b. Linearize: Δx �?J·Δq
   c. Solve QP: min ||J·Δq - (target - FK(q))||²
               s.t. q_min �?q + Δq �?q_max
   d. Update: q �?q + α·Δq (with line search)
```

### 5. Build System
**Dependency Management**:
- Git submodules for: Eigen, pugixml, DaQP, spdlog, nanobind, googletest
- Custom install script for system-level dependencies (Emscripten)

**CMake Structure**:
```
kinex/
├── CMakeLists.txt (root)
├── cmake/
�?  ├── Dependencies.cmake
�?  └── CompilerFlags.cmake
├── src/
�?  └── CMakeLists.txt
├── python/
�?  └── CMakeLists.txt
├── wasm/
�?  └── CMakeLists.txt
└── tests/
    └── CMakeLists.txt
```

**CMake Features**:
- Exported targets: `kinex::kinex`
- Install support with CMake config files
- Separate build options for Python/WASM bindings
- C++20 standard enforcement

### 6. Language Bindings

#### Python (nanobind)
**Design Decisions**:
- Use nanobind for smaller binary size vs pybind11
- Mirror C++ API with Pythonic naming (snake_case)
- Support NumPy arrays for joint angles and poses
- Provide type stubs for IDE support

**API Example**:
```python
import kinex
robot = kinex.Robot.from_urdf("ur5e.urdf")
fk = kinex.ForwardKinematics(robot)
pose = fk.compute([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
```

#### WebAssembly (Emscripten)
**Design Decisions**:
- Use Embind for C++ to JS binding
- Expose essential API only (minimize binary size)
- Return plain JS objects (not proxies) for performance
- Provide TypeScript definitions

**Build Optimizations**:
- Use `-O3` optimization
- Enable SIMD for Eigen operations
- Use `-s MODULARIZE=1` for clean module export

### 7. Visualization App
**Architecture**:
```
Three.js App
├── RobotRenderer (Three.js scene management)
├── URDFLoader (parse URDF, create meshes)
├── IKController (UI �?kinex WASM �?renderer)
└── UI Components (sliders, target controls)
```

**Technology Stack**:
- Three.js for 3D rendering
- React for UI components
- kinex WASM module for kinematics
- Vite for build system

**Features**:
- Load URDF with mesh visualization
- Interactive joint sliders
- IK target manipulation (drag end-effector)
- Real-time FK/IK updates

### 8. Testing Strategy

**C++ Tests (GTest)**:
- Unit tests for each component
- Integration tests for FK �?Jacobian �?IK pipeline
- Test fixtures using UR5e URDF
- Performance benchmarks

**Python Tests (pytest)**:
- API correctness tests
- NumPy array interop tests
- Memory leak tests (using gc)
- Performance comparison with C++

**JS Tests (Jest + Puppeteer)**:
- WASM module loading tests
- API correctness tests
- Browser integration tests
- Performance tests

## Data Flow

### Typical IK Solving Flow:
```
1. User provides URDF file
   �?
2. URDFParser �?Robot model
   �?
3. User specifies target pose
   �?
4. IKSolver initialization:
   - Build KinematicChain
   - Create ADFun tape for Jacobian
   - Configure DaQP solver
   �?
5. SQP iterations:
   - Compute FK(q) �?current pose
   - Compute J(q) �?Jacobian
   - Solve QP �?Δq
   - Update q
   �?
6. Return solution q
```

## Error Handling
- Use exceptions for unrecoverable errors (e.g., invalid URDF)
- Return std::optional or status codes for recoverable errors (e.g., IK not converged)
- Provide detailed error messages for debugging
- Use spdlog for all logging (warnings, errors, debug information)
- Log levels: DEBUG (detailed tracing), INFO (general information), WARN (warnings), ERROR (errors)
- Logging infrastructure must be set up before any C++ code implementation

## Performance Considerations
- Pre-allocate matrices for FK/Jacobian computation
- Reuse cached kinematic chain across IK iterations
- Warm-start IK solver for trajectory generation
- Use move semantics for large data structures
- Profile with perf/valgrind before optimization

## Security Considerations
- Validate URDF input to prevent XML injection
- Sanitize file paths for mesh loading
- Limit recursion depth in kinematic chain traversal
- Bounds checking for array accesses

## Future Extensions
- Collision detection using FCL
- Dynamics computation
- Multi-threaded IK solving
- GPU acceleration for Jacobian computation
- ROS2 integration
