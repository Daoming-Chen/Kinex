# Implementation Tasks

## Phase 1: Core C++ Refactoring - Robot → RobotModel (Breaking Change)

### 1.1 Header Files - Rename Robot to RobotModel
- [ ] 1.1.1 `core/include/kinex/robot_model.h`
  - [ ] Rename `class Robot` → `class RobotModel`
  - [ ] Update constructor: `Robot(const std::string& name)` → `RobotModel(const std::string& name)`
  - [ ] Update all method documentation
  - [ ] Verify KINEX_API export macro is present

- [ ] 1.1.2 `core/include/kinex/urdf_parser.h`
  - [ ] Change return type: `std::shared_ptr<Robot>` → `std::shared_ptr<RobotModel>`
  - [ ] Update documentation strings

- [ ] 1.1.3 `core/include/kinex/kinematics.h`
  - [ ] Update `KinematicChain` constructor parameter: `std::shared_ptr<const Robot>` → `std::shared_ptr<const RobotModel>`
  - [ ] Update `ForwardKinematics` constructor parameter similarly
  - [ ] Update `JacobianCalculator` constructor parameter similarly
  - [ ] Update all private member variables `robot_` type
  - [ ] Update documentation

- [ ] 1.1.4 `core/include/kinex/inverse_kinematics.h`
  - [ ] Update `IKSolver` constructor parameter: `std::shared_ptr<const Robot>` → `std::shared_ptr<const RobotModel>`
  - [ ] Update `SQPIKSolver` constructor parameter similarly
  - [ ] Update member variable `robot_` type
  - [ ] Update documentation

### 1.2 Implementation Files - Update Robot → RobotModel
- [ ] 1.2.1 `core/src/robot_model.cpp`
  - [ ] Rename all `Robot::` → `RobotModel::`
  - [ ] Update constructor implementation
  - [ ] Verify all method implementations
  - [ ] Run clang-format

- [ ] 1.2.2 `core/src/urdf_parser.cpp`
  - [ ] Update all `std::shared_ptr<Robot>` → `std::shared_ptr<RobotModel>`
  - [ ] Update `std::make_shared<Robot>` → `std::make_shared<RobotModel>`
  - [ ] Update variable names if they reference "robot" to "model" for clarity (optional but recommended)

- [ ] 1.2.3 `core/src/kinematics.cpp`
  - [ ] Update all template parameters and variable types
  - [ ] Ensure `std::shared_ptr<const RobotModel>` is used correctly
  - [ ] Update any debug/log messages that mention "Robot"

- [ ] 1.2.4 `core/src/inverse_kinematics.cpp`
  - [ ] Update all Robot references to RobotModel
  - [ ] Verify solver initialization code

### 1.3 C++ Tests - Update to Use RobotModel
- [ ] 1.3.1 `core/tests/test_urdf_parser.cpp`
  - [ ] Update variable declarations: `std::shared_ptr<Robot>` → `std::shared_ptr<RobotModel>`
  - [ ] Update test assertions and expectations
  - [ ] Verify all tests pass

- [ ] 1.3.2 `core/tests/test_kinematics.cpp`
  - [ ] Update `simple_robot_` and `ur5e_robot_` member variables to `std::shared_ptr<RobotModel>`
  - [ ] Update all test methods
  - [ ] Verify FK/Jacobian tests pass

- [ ] 1.3.3 `core/tests/test_inverse_kinematics.cpp`
  - [ ] Update robot model variables
  - [ ] Update IK solver instantiation
  - [ ] Verify all IK tests pass

## Phase 2: New Unified Robot Class Implementation

### 2.1 Create Robot Class Header
- [ ] 2.1.1 Create `core/include/kinex/robot.h`
  - [ ] Add include guards and necessary includes
  - [ ] Declare `class KINEX_API Robot` with proper export macro
  - [ ] Add static factory methods:
    - [ ] `static Robot fromURDF(const std::string& filepath, const std::string& end_link, const std::string& base_link = "")`
    - [ ] `static Robot fromURDFString(const std::string& urdf_string, const std::string& end_link, const std::string& base_link = "")`
  - [ ] Add copy/move constructors and assignment operators (or delete as appropriate)
  - [ ] Add `Robot clone() const` method
  - [ ] Add Forward Kinematics methods:
    - [ ] `Transform forwardKinematics(const Eigen::VectorXd& q, const std::string& link = "") const`
    - [ ] `Transform computePose(const Eigen::VectorXd& q, const std::string& link = "") const` (alias)
  - [ ] Add Inverse Kinematics methods:
    - [ ] `std::pair<Eigen::VectorXd, SolverStatus> inverseKinematics(const Transform& target, const Eigen::VectorXd& q_init, const std::string& link = "")`
    - [ ] `std::pair<Eigen::VectorXd, SolverStatus> solveIK(...)` (alias)
  - [ ] Add Jacobian methods:
    - [ ] `Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q, const std::string& link = "", JacobianType type = JacobianType::Analytic) const`
    - [ ] `double getManipulability(const Eigen::VectorXd& q, const std::string& link = "") const`
    - [ ] `bool isSingular(const Eigen::VectorXd& q, double threshold = 1e-6, const std::string& link = "") const`
    - [ ] `double getConditionNumber(const Eigen::VectorXd& q, const std::string& link = "") const`
  - [ ] Add IK configuration methods:
    - [ ] `void setIKTolerance(double tol)`
    - [ ] `void setPositionOnlyIK(bool enable)`
    - [ ] `void setOrientationOnlyIK(bool enable)`
    - [ ] `void setSolverConfig(const SolverConfig& config)`
    - [ ] `SolverConfig getSolverConfig() const`
  - [ ] Add accessor methods:
    - [ ] `std::shared_ptr<const RobotModel> getRobotModel() const`
    - [ ] `const std::string& getName() const`
    - [ ] `const std::string& getEndLink() const`
    - [ ] `const std::string& getBaseLink() const`
    - [ ] `size_t getDOF() const`
  - [ ] Add private members:
    - [ ] `std::shared_ptr<const RobotModel> model_`
    - [ ] `std::string end_link_`
    - [ ] `std::string base_link_`
    - [ ] `mutable std::unique_ptr<ForwardKinematics> fk_` (lazy init)
    - [ ] `mutable std::unordered_map<std::string, std::unique_ptr<ForwardKinematics>> fk_cache_`
    - [ ] `mutable std::unique_ptr<JacobianCalculator> jacobian_` (lazy init)
    - [ ] `mutable std::unordered_map<std::string, std::unique_ptr<JacobianCalculator>> jacobian_cache_`
    - [ ] `mutable std::unique_ptr<SQPIKSolver> ik_solver_` (lazy init)
    - [ ] `mutable std::unordered_map<std::string, std::unique_ptr<SQPIKSolver>> ik_cache_`
  - [ ] Add helper methods:
    - [ ] `const std::string& resolveLink(const std::string& link) const` (returns end_link_ if empty)
    - [ ] `ForwardKinematics& ensureFK(const std::string& link) const` (lazy initialization)
    - [ ] Similar helpers for Jacobian and IK
  - [ ] Add comprehensive doxygen documentation for all public methods

### 2.2 Implement Robot Class
- [ ] 2.2.1 Create `core/src/robot.cpp`
  - [ ] Implement static factory methods:
    - [ ] `fromURDF`: Use URDFParser, construct Robot from RobotModel
    - [ ] `fromURDFString`: Similar to fromURDF
  - [ ] Implement constructor (private, called by factories)
  - [ ] Implement `clone()`:
    - [ ] Deep copy RobotModel using copy constructor or clone method
    - [ ] Return new Robot with copied model, same end_link and base_link
    - [ ] Do NOT copy cached solvers (start fresh)
  - [ ] Implement FK methods:
    - [ ] `forwardKinematics`: Call ensureFK, then fk.compute()
    - [ ] `computePose`: inline delegate to forwardKinematics
  - [ ] Implement IK methods:
    - [ ] `inverseKinematics`: Lazy init IK solver, call solve(), return pair
    - [ ] `solveIK`: inline delegate
  - [ ] Implement Jacobian methods:
    - [ ] `computeJacobian`: Lazy init calculator, compute
    - [ ] `getManipulability`, `isSingular`, `getConditionNumber`: delegate to JacobianCalculator
  - [ ] Implement configuration methods:
    - [ ] `setIKTolerance`: Get config, modify, set back
    - [ ] `setPositionOnlyIK`, `setOrientationOnlyIK`: delegate to solver
    - [ ] `setSolverConfig`, `getSolverConfig`: direct access
  - [ ] Implement accessors:
    - [ ] Simple getters for model_, end_link_, base_link_, etc.
  - [ ] Implement helper methods:
    - [ ] `resolveLink`: return end_link_ if link.empty(), else link
    - [ ] `ensureFK`: Check cache, create if needed, return reference
  - [ ] Run clang-format on completed file

### 2.3 Update Build System
- [ ] 2.3.1 `core/CMakeLists.txt`
  - [ ] Add `src/robot.cpp` to the SOURCES list
  - [ ] Ensure `include/kinex/robot.h` is in public headers
  - [ ] Verify CMake configuration

- [ ] 2.3.2 Verify build on all platforms
  - [ ] Build on Linux: `cmake -B build && cmake --build build`
  - [ ] Build on macOS (if available)
  - [ ] Build on Windows (if available)
  - [ ] Verify no compiler warnings

### 2.4 Create C++ Tests for Robot Class
- [ ] 2.4.1 Create `core/tests/test_robot.cpp`
  - [ ] Test `Robot::fromURDF` with valid URDF
  - [ ] Test `Robot::fromURDFString`
  - [ ] Test `clone()` creates independent copy
  - [ ] Test FK methods match ForwardKinematics directly
  - [ ] Test IK methods match SQPIKSolver directly
  - [ ] Test Jacobian methods match JacobianCalculator directly
  - [ ] Test default end-effector usage
  - [ ] Test per-call link override
  - [ ] Test both method name aliases (forwardKinematics vs computePose)
  - [ ] Test IK configuration persistence
  - [ ] Test cloned robots have independent solver configs
  - [ ] Test thread safety of cloned robots (parallel FK/IK calls)
  - [ ] Test lazy initialization (no solver created until first use)
  - [ ] Test error handling (invalid link names, etc.)

- [ ] 2.4.2 Run all C++ tests
  - [ ] `ctest --test-dir build --output-on-failure`
  - [ ] Verify all tests pass
  - [ ] Check test coverage if available

## Phase 3: Python Bindings Update

### 3.1 Update Python Bindings for RobotModel and Robot
- [ ] 3.1.1 `bindings/python/src/bindings.cpp`
  - [ ] Update existing `Robot` class binding to `RobotModel`:
    - [ ] Change `nb::class_<Robot>(m, "Robot")` → `nb::class_<RobotModel>(m, "RobotModel")`
    - [ ] Update all method bindings
  - [ ] Add new `Robot` class binding:
    - [ ] `nb::class_<Robot>(m, "Robot")`
    - [ ] Bind static methods: `from_urdf`, `from_urdf_string`
    - [ ] Bind `clone()` method
    - [ ] Bind FK methods: `forward_kinematics`, `compute_pose`
    - [ ] Bind IK methods: `inverse_kinematics`, `solve_ik` (return tuple)
    - [ ] Bind Jacobian methods: `compute_jacobian`, `get_manipulability`, `is_singular`, `get_condition_number`
    - [ ] Bind configuration methods: `set_ik_tolerance`, `set_position_only_ik`, etc.
    - [ ] Bind accessors as properties: `name`, `dof`, `end_link`, `base_link`, `joint_names`
    - [ ] Ensure NumPy array conversions work (Eigen::VectorXd ↔ np.ndarray)
    - [ ] Release GIL for compute-intensive operations

### 3.2 Update Python Package Files
- [ ] 3.2.1 `bindings/python/kinex/__init__.py`
  - [ ] Ensure both `RobotModel` and `Robot` are exported
  - [ ] Update any examples/docstrings

- [ ] 3.2.2 `bindings/python/kinex/__init__.pyi`
  - [ ] Add type stubs for `RobotModel` class (renamed from Robot)
  - [ ] Add type stubs for new `Robot` class
  - [ ] Define method signatures with proper type hints
  - [ ] Use `numpy.typing.NDArray` for array types

### 3.3 Update Python Tests
- [ ] 3.3.1 `bindings/python/tests/test_robot_model.py`
  - [ ] Update to use `kinex.RobotModel` instead of `kinex.Robot`
  - [ ] Update `Robot.from_urdf_file` → `RobotModel.from_urdf_file`
  - [ ] Verify all tests pass

- [ ] 3.3.2 `bindings/python/tests/test_forward_kinematics.py`
  - [ ] Update robot loading to use `RobotModel`
  - [ ] Verify ForwardKinematics accepts RobotModel

- [ ] 3.3.3 `bindings/python/tests/test_jacobian.py`
  - [ ] Update to use RobotModel
  - [ ] Verify JacobianCalculator works

- [ ] 3.3.4 `bindings/python/tests/test_inverse_kinematics.py`
  - [ ] Update to use RobotModel
  - [ ] Verify SQPIKSolver works

- [ ] 3.3.5 Create `bindings/python/tests/test_robot.py`
  - [ ] Test `Robot.from_urdf()`
  - [ ] Test `Robot.from_urdf_string()`
  - [ ] Test `clone()` method
  - [ ] Test FK methods with NumPy arrays
  - [ ] Test IK methods return correct tuple
  - [ ] Test Jacobian methods return NumPy arrays
  - [ ] Test manipulability methods
  - [ ] Test configuration methods
  - [ ] Test properties (name, dof, etc.)
  - [ ] Test both method name variants
  - [ ] Test thread safety with threading module

- [ ] 3.3.6 `bindings/python/tests/conftest.py`
  - [ ] Update fixtures if they load Robot (change to RobotModel or new Robot as appropriate)

### 3.4 Run Python Tests
- [ ] 3.4.1 Install updated bindings: `pip install -e bindings/python`
- [ ] 3.4.2 Run pytest: `pytest bindings/python/tests/`
- [ ] 3.4.3 Verify all tests pass
- [ ] 3.4.4 Check for any deprecation warnings

## Phase 4: WASM Bindings Update

### 4.1 Update WASM Bindings
- [ ] 4.1.1 `bindings/wasm/src/bindings.cpp`
  - [ ] Update `RobotHandle` internal type:
    - [ ] Change `std::shared_ptr<kinex::Robot> robot_` → `std::shared_ptr<kinex::RobotModel> robot_`
    - [ ] Update all internal usages
  - [ ] Add new `RobotWrapper` class (or modify existing):
    - [ ] Wrapper for C++ Robot class
    - [ ] Implement fromURDFString static method
    - [ ] Implement forwardKinematics, computePose (return PoseData)
    - [ ] Implement inverseKinematics, solveIK (return IKResultData)
    - [ ] Implement computeJacobian (return MatrixData)
    - [ ] Implement manipulability methods
    - [ ] Implement configuration methods
    - [ ] Implement clone()
    - [ ] Implement property getters (getName, getDOF, etc.)
  - [ ] Bind new Robot class using Embind:
    - [ ] `class_<RobotWrapper>("Robot")`
    - [ ] `.class_function("fromURDFString", ...)`
    - [ ] `.function("forwardKinematics", ...)` with optional overrides for JS types
    - [ ] `.function("computePose", ...)` (alias)
    - [ ] `.function("inverseKinematics", ...)` with val conversions
    - [ ] `.function("solveIK", ...)` (alias)
    - [ ] `.function("computeJacobian", ...)`
    - [ ] `.function("getManipulability", ...)`
    - [ ] `.function("isSingular", ...)`
    - [ ] `.function("getConditionNumber", ...)`
    - [ ] `.function("setIKTolerance", ...)`, etc.
    - [ ] `.function("clone", ...)`
    - [ ] `.function("getName", ...)`, `.function("getDOF", ...)`, etc.

### 4.2 Update TypeScript Definitions
- [ ] 4.2.1 `bindings/wasm/kinex.d.ts`
  - [ ] Update RobotHandle (if exposed) to reflect internal changes
  - [ ] Add new `Robot` class declaration:
    - [ ] `class Robot`
    - [ ] `static fromURDFString(urdf: string, endLink: string, baseLink?: string): Robot`
    - [ ] `forwardKinematics(jointAngles: number[], link?: string): Pose`
    - [ ] `computePose(jointAngles: number[], link?: string): Pose` (alias)
    - [ ] `inverseKinematics(targetPose: Pose, initialGuess: number[], link?: string): IKResult`
    - [ ] `solveIK(...)` (alias)
    - [ ] `computeJacobian(jointAngles: number[], link?: string, type?: JacobianType): Matrix`
    - [ ] `getManipulability(jointAngles: number[], link?: string): number`
    - [ ] `isSingular(jointAngles: number[], threshold?: number, link?: string): boolean`
    - [ ] `getConditionNumber(jointAngles: number[], link?: string): number`
    - [ ] `setIKTolerance(tolerance: number): void`, etc.
    - [ ] `clone(): Robot`
    - [ ] `getName(): string`, `getDOF(): number`, etc.

### 4.3 Update WASM Tests
- [ ] 4.3.1 `bindings/wasm/tests/node/full_bindings.test.js`
  - [ ] Update tests to use RobotModel internally (via RobotHandle)
  - [ ] Add tests for new Robot class
  - [ ] Test fromURDFString
  - [ ] Test FK, IK, Jacobian methods
  - [ ] Test clone()
  - [ ] Verify TypedArray conversions

- [ ] 4.3.2 `bindings/wasm/tests/browser/module.browser.test.js`
  - [ ] Update similar to node tests
  - [ ] Test in browser environment

### 4.4 Build and Test WASM
- [ ] 4.4.1 Build WASM: `npm run build` (in bindings/wasm/)
- [ ] 4.4.2 Run Node tests: `npm test`
- [ ] 4.4.3 Run browser tests (manual or automated)
- [ ] 4.4.4 Verify WASM bundle size hasn't increased significantly

## Phase 5: Examples Update

### 5.1 Python Examples
- [ ] 5.1.1 `examples/python/forward_kinematics.py`
  - [ ] Update to use new `Robot` class:
    - [ ] `robot = kinex.Robot.from_urdf("ur5e.urdf", "wrist_3_link")`
    - [ ] `pose = robot.forward_kinematics(q_home)`
  - [ ] Test example runs successfully

- [ ] 5.1.2 `examples/python/inverse_kinematics.py`
  - [ ] Update to use `Robot` class and `robot.inverse_kinematics()`
  - [ ] Test example runs

- [ ] 5.1.3 `examples/python/jacobian_analysis.py`
  - [ ] Update to use `Robot` class and `robot.compute_jacobian()`
  - [ ] Test manipulability methods

- [ ] 5.1.4 `examples/python/trajectory_generation.py`
  - [ ] Update to use `Robot` class
  - [ ] Test trajectory planning works

### 5.2 C++ Examples
- [ ] 5.2.1 `examples/cpp/forward_kinematics_example.cpp`
  - [ ] Update includes: `#include <kinex/robot.h>`
  - [ ] Update code to use `Robot::fromURDF()`
  - [ ] Use `robot.forwardKinematics(q)`
  - [ ] Rebuild example: check `examples/cpp/CMakeLists.txt` if needed
  - [ ] Test example runs

### 5.3 JavaScript Examples
- [ ] 5.3.1 `examples/javascript/app.js`
  - [ ] Update to use new `Robot` class in WASM
  - [ ] `const robot = kinex.Robot.fromURDFString(urdfContent, "tool0")`
  - [ ] Use `robot.forwardKinematics(jointAngles)`
  - [ ] Test in browser or Node.js

## Phase 6: Documentation Update

### 6.1 README Files
- [ ] 6.1.1 `README.md`
  - [ ] Update Python quick start example to use `Robot` class
  - [ ] Update JavaScript example to use `Robot` class
  - [ ] Add note about breaking change (v2.0)
  - [ ] Mention both `Robot` (high-level) and `RobotModel` (low-level) are available

- [ ] 6.1.2 `README_zh.md`
  - [ ] Update Chinese examples similarly
  - [ ] Translate breaking change notes

### 6.2 API Documentation
- [ ] 6.2.1 `docs/api/python.md`
  - [ ] Document `kinex.RobotModel` class (low-level)
  - [ ] Document `kinex.Robot` class (recommended, high-level)
  - [ ] Document all methods with examples
  - [ ] Clarify when to use Robot vs RobotModel

- [ ] 6.2.2 Create `docs/api/cpp.md` (if doesn't exist)
  - [ ] Document `kinex::RobotModel` class
  - [ ] Document `kinex::Robot` class
  - [ ] Provide C++ code examples

### 6.3 Tutorials
- [ ] 6.3.1 `docs/tutorials/cpp-tutorial.md`
  - [ ] Update examples to use `Robot::fromURDF()`
  - [ ] Show both high-level and low-level API usage

- [ ] 6.3.2 Update other tutorials as needed

### 6.4 Guides
- [ ] 6.4.1 `docs/guides/getting-started.md`
  - [ ] Update quick start to feature `Robot` class
  - [ ] Add section on migration from v1.x to v2.0

- [ ] 6.4.2 Create `docs/guides/migration-v2.md`
  - [ ] Explain Robot → RobotModel rename
  - [ ] Show before/after code examples
  - [ ] Provide migration checklist for users
  - [ ] Explain benefits of new unified API

## Phase 7: Benchmarks Update

### 7.1 Update Benchmark Scripts
- [ ] 7.1.1 `benchmarks/run_tier_a_benchmarks.py`
  - [ ] Update imports if needed
  - [ ] Change `kinex.Robot` → `kinex.RobotModel` or use new `Robot` class
  - [ ] Verify benchmarks run and produce valid results
  - [ ] Compare results with baseline to ensure no regression

- [ ] 7.1.2 `benchmarks/run_tier_b_benchmarks.py`
  - [ ] Update similarly
  - [ ] Test synthetic robot generation

- [ ] 7.1.3 `benchmarks/oracle.py`
  - [ ] Update robot loading
  - [ ] Verify oracle validation works

- [ ] 7.1.4 `benchmarks/urdf_generator.py`
  - [ ] Update if it creates Robot instances
  - [ ] Verify generated URDFs work with RobotModel

### 7.2 Run Full Benchmark Suite
- [ ] 7.2.1 Run: `python benchmarks/run_benchmarks.py --all`
- [ ] 7.2.2 Verify no performance regressions
- [ ] 7.2.3 Update benchmark results if needed

## Phase 8: Final Integration and Testing

### 8.1 Full Build Test
- [ ] 8.1.1 Clean build from scratch
  - [ ] `rm -rf build && cmake -B build -DCMAKE_BUILD_TYPE=Release`
  - [ ] `cmake --build build -j`
- [ ] 8.1.2 Install and test
  - [ ] `sudo cmake --install build --prefix /usr/local`
  - [ ] Create test project using installed kinex
  - [ ] Verify both Robot and RobotModel are accessible

### 8.2 Run All Tests
- [ ] 8.2.1 C++ tests: `ctest --test-dir build --output-on-failure`
- [ ] 8.2.2 Python tests: `pytest bindings/python/tests/`
- [ ] 8.2.3 WASM tests: `npm test` in bindings/wasm/
- [ ] 8.2.4 Verify all tests pass

### 8.3 Code Quality Checks
- [ ] 8.3.1 Run clang-format on all modified C++ files
- [ ] 8.3.2 Run clang-tidy if configured
- [ ] 8.3.3 Check for memory leaks with valgrind or sanitizers
- [ ] 8.3.4 Verify no compiler warnings

### 8.4 Documentation Validation
- [ ] 8.4.1 Build documentation if using Doxygen/Sphinx
- [ ] 8.4.2 Verify all code examples in docs compile and run
- [ ] 8.4.3 Check for broken links

### 8.5 Version and Changelog
- [ ] 8.5.1 Update version to 2.0.0 in all relevant files
  - [ ] CMakeLists.txt
  - [ ] setup.py
  - [ ] package.json
- [ ] 8.5.2 Update CHANGELOG.md
  - [ ] Document breaking changes
  - [ ] Document new Robot class features
  - [ ] Provide migration guide link

## Phase 9: Pre-Release Validation

### 9.1 Platform Testing
- [ ] 9.1.1 Test on Linux (Ubuntu 20.04+, Fedora)
- [ ] 9.1.2 Test on macOS (11+)
- [ ] 9.1.3 Test on Windows (10/11)
- [ ] 9.1.4 Test WASM in Chrome, Firefox, Safari

### 9.2 Create Migration Examples
- [ ] 9.2.1 Create example showing v1.x → v2.0 migration
- [ ] 9.2.2 Add to docs/guides/migration-v2.md

### 9.3 Final Review
- [ ] 9.3.1 Review all changed files for consistency
- [ ] 9.3.2 Ensure all TODOs and FIXMEs are resolved
- [ ] 9.3.3 Verify LICENSE headers on new files
- [ ] 9.3.4 Run final spell check on documentation

## Summary
- **Total Tasks**: ~150+ individual checklist items
- **Estimated Files Modified**: 40+
- **New Files Created**: ~8
- **Breaking Change**: Yes (major version 2.0)
- **Testing Required**: C++, Python, WASM on multiple platforms
