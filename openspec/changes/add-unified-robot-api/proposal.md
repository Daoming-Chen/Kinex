# Change: Add Unified Robot API

## Why
The current C++ API requires users to instantiate and manage separate classes (`Robot`, `URDFParser`, `ForwardKinematics`, `JacobianCalculator`, `IKSolver`) to perform common robotics operations. The existing `Robot` class is actually just a data structure representing the robot's structural model (links and joints), not a fully operational robot interface. This creates:

1. **Fragmented user experience**: Users must understand and coordinate 4-5 different classes
2. **Unclear naming**: "Robot" suggests a complete robot, but it's just the model structure
3. **Repetitive boilerplate**: Every operation requires creating solver objects
4. **No convenient cloning**: Multi-threaded applications need complex manual copying

This change creates a unified `Robot` class that provides simple, intuitive access to all kinematics operations while clarifying the architecture by renaming the structural model to `RobotModel`.

## What Changes

### Core Architecture Refactoring
- **Rename** `Robot` class → `RobotModel` throughout entire codebase
  - This clarifies that this class represents only the structural model (links, joints, geometry)
  - All references in C++ headers, implementations, tests, bindings, examples updated

### New Unified Robot Class
- **Create** new `Robot` class as high-level unified interface providing:
  - **Static factories**: `Robot::fromURDF(path, end_link)`, `Robot::fromURDFString(xml, end_link)`
  - **Cloning**: `robot.clone()` for deep copy enabling multi-threaded usage
  - **Forward kinematics**: `forwardKinematics(q)`, `computePose(q, link)`
  - **Inverse kinematics**: `inverseKinematics(target, q_init)`, `solveIK(...)`
  - **Jacobian**: `computeJacobian(q)`, `getManipulability(q)`, `isSingular(q)`, `getConditionNumber(q)`
  - **Configuration**: `setIKTolerance()`, `setPositionOnlyIK()`, `setSolverConfig()`, etc.
  - **Default end-effector**: Configured at construction, overridable per method call

### Implementation Details
- `Robot` class uses **Facade pattern** with lazy initialization
- Internally holds `std::shared_ptr<const RobotModel>` (immutable)
- Lazily constructs `ForwardKinematics`, `JacobianCalculator`, `SQPIKSolver` on first use
- `clone()` performs deep copy of RobotModel only (not solver state)

### Compatibility
- **API Refactoring**: All code using `Robot` must change to `RobotModel`
- Low-level classes (`ForwardKinematics`, etc.) remain accessible for advanced users
- This establishes the standard API for v1.0

## Impact

### Affected Specifications
- `urdf-parsing` - Modified to return `RobotModel`, added `Robot` factories
- `forward-kinematics` - Added unified `Robot` API methods
- `inverse-kinematics` - Added unified `Robot` API methods
- `jacobian-computation` - Added unified `Robot` API methods
- `python-bindings` - Updated all bindings, added new `Robot` class
- `wasm-bindings` - Updated all bindings, added new `Robot` class
- `build-system` - Added new source files
- `benchmark-infrastructure` - Updated to use `RobotModel`

### Affected Files (40+ files)

#### C++ Core (11 files)
- `core/include/kinex/robot_model.h` - **MODIFIED**: Rename `Robot` → `RobotModel`
- `core/include/kinex/robot.h` - **NEW**: Unified `Robot` class declaration
- `core/include/kinex/urdf_parser.h` - **MODIFIED**: Return `RobotModel` instead of `Robot`
- `core/include/kinex/kinematics.h` - **MODIFIED**: Accept `RobotModel` instead of `Robot`
- `core/include/kinex/inverse_kinematics.h` - **MODIFIED**: Accept `RobotModel` instead of `Robot`
- `core/src/robot_model.cpp` - **MODIFIED**: Class name `Robot` → `RobotModel`
- `core/src/robot.cpp` - **NEW**: Unified `Robot` class implementation
- `core/src/urdf_parser.cpp` - **MODIFIED**: Use `RobotModel` instead of `Robot`
- `core/src/kinematics.cpp` - **MODIFIED**: Use `RobotModel` instead of `Robot`
- `core/src/inverse_kinematics.cpp` - **MODIFIED**: Use `RobotModel` instead of `Robot`
- `core/CMakeLists.txt` - **MODIFIED**: Add `robot.cpp` to build

#### C++ Tests (4 files)
- `core/tests/test_urdf_parser.cpp` - **MODIFIED**: Use `RobotModel`
- `core/tests/test_kinematics.cpp` - **MODIFIED**: Use `RobotModel`
- `core/tests/test_inverse_kinematics.cpp` - **MODIFIED**: Use `RobotModel`
- `core/tests/test_robot.cpp` - **NEW**: Test unified `Robot` class

#### Python Bindings (8 files)
- `bindings/python/src/bindings.cpp` - **MODIFIED**: Bind `RobotModel` (rename from `Robot`) and new `Robot` class
- `bindings/python/kinex/__init__.py` - **MODIFIED**: Export both `RobotModel` and `Robot`
- `bindings/python/kinex/__init__.pyi` - **MODIFIED**: Type stubs for both classes
- `bindings/python/tests/test_robot_model.py` - **MODIFIED**: Use `RobotModel`
- `bindings/python/tests/test_forward_kinematics.py` - **MODIFIED**: Use `RobotModel`
- `bindings/python/tests/test_jacobian.py` - **MODIFIED**: Use `RobotModel`
- `bindings/python/tests/test_inverse_kinematics.py` - **MODIFIED**: Use `RobotModel`
- `bindings/python/tests/test_robot.py` - **NEW**: Test unified `Robot` API

#### WASM Bindings (4 files)
- `bindings/wasm/src/bindings.cpp` - **MODIFIED**: `RobotHandle` uses `RobotModel`, add new `Robot` bindings
- `bindings/wasm/kinex.d.ts` - **MODIFIED**: TypeScript definitions
- `bindings/wasm/tests/node/full_bindings.test.js` - **MODIFIED**: Update tests
- `bindings/wasm/tests/browser/module.browser.test.js` - **MODIFIED**: Update tests

#### Examples (6 files)
- `examples/python/forward_kinematics.py` - **MODIFIED**: Use new `Robot` class
- `examples/python/inverse_kinematics.py` - **MODIFIED**: Use new `Robot` class
- `examples/python/jacobian_analysis.py` - **MODIFIED**: Use new `Robot` class
- `examples/python/trajectory_generation.py` - **MODIFIED**: Use new `Robot` class
- `examples/cpp/forward_kinematics_example.cpp` - **MODIFIED**: Use new `Robot` class
- `examples/javascript/app.js` - **MODIFIED**: Use new `Robot` class

#### Documentation (5 files)
- `README.md` - **MODIFIED**: Update code examples to use new `Robot` API
- `README_zh.md` - **MODIFIED**: Update Chinese examples
- `docs/api/python.md` - **MODIFIED**: Document `Robot` and `RobotModel` classes
- `docs/tutorials/cpp-tutorial.md` - **MODIFIED**: Update C++ examples
- `docs/guides/getting-started.md` - **MODIFIED**: Update quick start guide

#### Benchmarks (4 files)
- `benchmarks/run_tier_a_benchmarks.py` - **MODIFIED**: Update `Robot` usage
- `benchmarks/run_tier_b_benchmarks.py` - **MODIFIED**: Update `Robot` usage
- `benchmarks/oracle.py` - **MODIFIED**: Update `Robot` usage
- `benchmarks/urdf_generator.py` - **MODIFIED**: Update `Robot` usage if needed

### Migration Path
This is a **breaking change** requiring:
1. **Version bump**: 1.x → 2.0.0 (semantic versioning major bump)
2. **Migration guide**: Document how to update existing code
3. **Deprecation period**: Could provide compatibility shim if needed (but not recommended)

**Simple migration pattern**:
```cpp
// Old code (v1.x)
URDFParser parser;
auto robot = parser.parseFile("ur5e.urdf");
ForwardKinematics fk(robot, "tool0");
auto pose = fk.compute(q);

// New code (v2.0) - Option 1: Use unified API
auto robot = Robot::fromURDF("ur5e.urdf", "tool0");
auto pose = robot.forwardKinematics(q);

// New code (v2.0) - Option 2: Use low-level API
URDFParser parser;
auto model = parser.parseFile("ur5e.urdf");  // Returns RobotModel
ForwardKinematics fk(model, "tool0");
auto pose = fk.compute(q);
```

## Breaking Changes
**Yes - Major Breaking Change**:
- All code using `Robot` class must change to `RobotModel`
- This affects C++, Python, and JavaScript/WASM APIs
- Requires major version bump (2.0.0)
- User code will need updates, but migration is straightforward (mostly find-replace)

## Benefits
Despite breaking changes, this provides:
1. **Clearer architecture**: `RobotModel` (structure) vs `Robot` (operational interface)
2. **Better UX**: Single class for 90% of use cases
3. **Easier multi-threading**: Built-in `clone()` support
4. **More Pythonic/idiomatic**: Matches user expectations of a "Robot" class
5. **Better long-term maintainability**: Clear separation of concerns
