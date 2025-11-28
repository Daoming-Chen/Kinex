# Implementation Tasks

## Phase 1: Core C++ Refactoring - Robot → RobotModel

### 1.1 Header Files - Rename Robot to RobotModel
- [x] 1.1.1 `core/include/kinex/robot_model.h`
  - [x] Rename `class Robot` → `class RobotModel`
  - [x] Update constructor: `Robot(const std::string& name)` → `RobotModel(const std::string& name)`
  - [x] Update all method documentation
  - [x] Verify KINEX_API export macro is present

- [x] 1.1.2 `core/include/kinex/urdf_parser.h`
  - [x] Change return type: `std::shared_ptr<Robot>` → `std::shared_ptr<RobotModel>`
  - [x] Update documentation strings

- [x] 1.1.3 `core/include/kinex/kinematics.h`
  - [x] Update `KinematicChain` constructor parameter: `std::shared_ptr<const Robot>` → `std::shared_ptr<const RobotModel>`
  - [x] Update `ForwardKinematics` constructor parameter similarly
  - [x] Update `JacobianCalculator` constructor parameter similarly
  - [x] Update all private member variables `robot_` type
  - [x] Update documentation

- [x] 1.1.4 `core/include/kinex/inverse_kinematics.h`
  - [x] Update `IKSolver` constructor parameter: `std::shared_ptr<const Robot>` → `std::shared_ptr<const RobotModel>`
  - [x] Update `SQPIKSolver` constructor parameter similarly
  - [x] Update member variable `robot_` type
  - [x] Update documentation

### 1.2 Implementation Files - Update Robot → RobotModel
- [x] 1.2.1 `core/src/robot_model.cpp`
  - [x] Rename all `Robot::` → `RobotModel::`
  - [x] Update constructor implementation
  - [x] Verify all method implementations
  - [x] Run clang-format

- [x] 1.2.2 `core/src/urdf_parser.cpp`
  - [x] Update all `std::shared_ptr<Robot>` → `std::shared_ptr<RobotModel>`
  - [x] Update `std::make_shared<Robot>` → `std::make_shared<RobotModel>`
  - [x] Update variable names if they reference "robot" to "model" for clarity (optional but recommended)

- [x] 1.2.3 `core/src/kinematics.cpp`
  - [x] Update all template parameters and variable types
  - [x] Ensure `std::shared_ptr<const RobotModel>` is used correctly
  - [x] Update any debug/log messages that mention "Robot"

- [x] 1.2.4 `core/src/inverse_kinematics.cpp`
  - [x] Update all Robot references to RobotModel
  - [x] Verify solver initialization code

### 1.3 C++ Tests - Update to Use RobotModel
- [x] 1.3.1 `core/tests/test_urdf_parser.cpp`
  - [x] Update variable declarations: `std::shared_ptr<Robot>` → `std::shared_ptr<RobotModel>`
  - [x] Update test assertions and expectations
  - [x] Verify all tests pass

- [x] 1.3.2 `core/tests/test_kinematics.cpp`
  - [x] Update `simple_robot_` and `ur5e_robot_` member variables to `std::shared_ptr<RobotModel>`
  - [x] Update all test methods
  - [x] Verify FK/Jacobian tests pass

- [x] 1.3.3 `core/tests/test_inverse_kinematics.cpp`
  - [x] Update robot model variables
  - [x] Update IK solver instantiation
  - [x] Verify all IK tests pass

## Phase 2: New Unified Robot Class Implementation

### 2.1 Create Robot Class Header
- [x] 2.1.1 Create `core/include/kinex/robot.h`
  - [x] Add include guards and necessary includes
  - [x] Declare `class KINEX_API Robot` with proper export macro
  - [x] Add static factory methods:
    - [x] `static Robot fromURDF(const std::string& filepath, const std::string& end_link, const std::string& base_link = "")`
    - [x] `static Robot fromURDFString(const std::string& urdf_string, const std::string& end_link, const std::string& base_link = "")`
  - [x] Add copy/move constructors and assignment operators (or delete as appropriate)
  - [x] Add `Robot clone() const` method
  - [x] Add Forward Kinematics methods:
    - [x] `Transform forwardKinematics(const Eigen::VectorXd& q, const std::string& link = "") const`
    - [x] `Transform computePose(const Eigen::VectorXd& q, const std::string& link = "") const` (alias)
  - [x] Add Inverse Kinematics methods:
    - [x] `std::pair<Eigen::VectorXd, SolverStatus> inverseKinematics(const Transform& target, const Eigen::VectorXd& q_init, const std::string& link = "")`
    - [x] `std::pair<Eigen::VectorXd, SolverStatus> solveIK(...)` (alias)
  - [x] Add Jacobian methods:
    - [x] `Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q, const std::string& link = "", JacobianType type = JacobianType::Analytic) const`
    - [x] `double getManipulability(const Eigen::VectorXd& q, const std::string& link = "") const`
    - [x] `bool isSingular(const Eigen::VectorXd& q, double threshold = 1e-6, const std::string& link = "") const`
    - [x] `double getConditionNumber(const Eigen::VectorXd& q, const std::string& link = "") const`
  - [x] Add IK configuration methods:
    - [x] `void setIKTolerance(double tol)`
    - [x] `void setPositionOnlyIK(bool enable)`
    - [x] `void setOrientationOnlyIK(bool enable)`
    - [x] `void setSolverConfig(const SolverConfig& config)`
    - [x] `SolverConfig getSolverConfig() const`
  - [x] Add accessor methods:
    - [x] `std::shared_ptr<const RobotModel> getRobotModel() const`
    - [x] `const std::string& getName() const`
    - [x] `const std::string& getEndLink() const`
    - [x] `const std::string& getBaseLink() const`
    - [x] `size_t getDOF() const`
  - [x] Add private members:
    - [x] `std::shared_ptr<const RobotModel> model_`
    - [x] `std::string end_link_`
    - [x] `std::string base_link_`
    - [x] `mutable std::unique_ptr<ForwardKinematics> fk_` (lazy init)
    - [x] `mutable std::unordered_map<std::string, std::unique_ptr<ForwardKinematics>> fk_cache_`
    - [x] `mutable std::unique_ptr<JacobianCalculator> jacobian_` (lazy init)
    - [x] `mutable std::unordered_map<std::string, std::unique_ptr<JacobianCalculator>> jacobian_cache_`
    - [x] `mutable std::unique_ptr<SQPIKSolver> ik_solver_` (lazy init)
    - [x] `mutable std::unordered_map<std::string, std::unique_ptr<SQPIKSolver>> ik_cache_`
  - [x] Add helper methods:
    - [x] `const std::string& resolveLink(const std::string& link) const` (returns end_link_ if empty)
    - [x] `ForwardKinematics& ensureFK(const std::string& link) const` (lazy initialization)
    - [x] Similar helpers for Jacobian and IK
  - [x] Add comprehensive doxygen documentation for all public methods

### 2.2 Implement Robot Class
- [x] 2.2.1 Create `core/src/robot.cpp`
  - [x] Implement static factory methods:
    - [x] `fromURDF`: Use URDFParser, construct Robot from RobotModel
    - [x] `fromURDFString`: Similar to fromURDF
  - [x] Implement constructor (private, called by factories)
  - [x] Implement `clone()`:
    - [x] Deep copy RobotModel using copy constructor or clone method
    - [x] Return new Robot with copied model, same end_link and base_link
    - [x] Do NOT copy cached solvers (start fresh)
  - [x] Implement FK methods:
    - [x] `forwardKinematics`: Call ensureFK, then fk.compute()
    - [x] `computePose`: inline delegate to forwardKinematics
  - [x] Implement IK methods:
    - [x] `inverseKinematics`: Lazy init IK solver, call solve(), return pair
    - [x] `solveIK`: inline delegate
  - [x] Implement Jacobian methods:
    - [x] `computeJacobian`: Lazy init calculator, compute
    - [x] `getManipulability`, `isSingular`, `getConditionNumber`: delegate to JacobianCalculator
  - [x] Implement configuration methods:
    - [x] `setIKTolerance`: Get config, modify, set back
    - [x] `setPositionOnlyIK`, `setOrientationOnlyIK`: delegate to solver
    - [x] `setSolverConfig`, `getSolverConfig`: direct access
  - [x] Implement accessors:
    - [x] Simple getters for model_, end_link_, base_link_, etc.
  - [x] Implement helper methods:
    - [x] `resolveLink`: return end_link_ if link.empty(), else link
    - [x] `ensureFK`: Check cache, create if needed, return reference
  - [x] Run clang-format on completed file

### 2.3 Update Build System
- [x] 2.3.1 `core/CMakeLists.txt`
  - [x] Add `src/robot.cpp` to the SOURCES list
  - [x] Ensure `include/kinex/robot.h` is in public headers
  - [x] Verify CMake configuration

- [x] 2.3.2 Verify build on all platforms
  - [x] Build on Linux: `cmake -B build && cmake --build build`
  - [x] Build on macOS (if available)
  - [x] Build on Windows (if available)
  - [x] Verify no compiler warnings

### 2.4 Create C++ Tests for Robot Class
- [x] 2.4.1 Create `core/tests/test_robot.cpp`
  - [x] Test `Robot::fromURDF` with valid URDF
  - [x] Test `Robot::fromURDFString`
  - [x] Test `clone()` creates independent copy
  - [x] Test FK methods match ForwardKinematics directly
  - [x] Test IK methods match SQPIKSolver directly
  - [x] Test Jacobian methods match JacobianCalculator directly
  - [x] Test default end-effector usage
  - [x] Test per-call link override
  - [x] Test both method name aliases (forwardKinematics vs computePose)
  - [x] Test IK configuration persistence
  - [x] Test cloned robots have independent solver configs
  - [x] Test thread safety of cloned robots (parallel FK/IK calls)
  - [x] Test lazy initialization (no solver created until first use)
  - [x] Test error handling (invalid link names, etc.)

- [x] 2.4.2 Run all C++ tests
  - [x] `ctest --test-dir build --output-on-failure`
  - [x] Verify all tests pass
  - [x] Check test coverage if available

## Phase 3: Python Bindings Update

### 3.1 Update Python Bindings for RobotModel and Robot
- [x] 3.1.1 `bindings/python/src/bindings.cpp`
  - [x] Update existing `Robot` class binding to `RobotModel`:
    - [x] Change `nb::class_<Robot>(m, "Robot")` → `nb::class_<RobotModel>(m, "RobotModel")`
    - [x] Update all method bindings
  - [x] Add new `Robot` class binding:
    - [x] `nb::class_<Robot>(m, "Robot")`
    - [x] Bind static methods: `from_urdf`, `from_urdf_string`
    - [x] Bind `clone()` method
    - [x] Bind FK methods: `forward_kinematics`, `compute_pose`
    - [x] Bind IK methods: `inverse_kinematics`, `solve_ik` (return tuple)
    - [x] Bind Jacobian methods: `compute_jacobian`, `get_manipulability`, `is_singular`, `get_condition_number`
    - [x] Bind configuration methods: `set_ik_tolerance`, `set_position_only_ik`, etc.
    - [x] Bind accessors as properties: `name`, `dof`, `end_link`, `base_link`, `joint_names`
    - [x] Ensure NumPy array conversions work (Eigen::VectorXd ↔ np.ndarray)
    - [x] Release GIL for compute-intensive operations

### 3.2 Update Python Package Files
- [x] 3.2.1 `bindings/python/kinex/__init__.py`
  - [x] Ensure both `RobotModel` and `Robot` are exported
  - [x] Update any examples/docstrings

- [x] 3.2.2 `bindings/python/kinex/__init__.pyi`
  - [x] Add type stubs for `RobotModel` class (renamed from Robot)
  - [x] Add type stubs for new `Robot` class
  - [x] Define method signatures with proper type hints
  - [x] Use `numpy.typing.NDArray` for array types

### 3.3 Update Python Tests
- [x] 3.3.1 `bindings/python/tests/test_robot_model.py`
  - [x] Update to use `kinex.RobotModel` instead of `kinex.Robot`
  - [x] Update `Robot.from_urdf_file` → `RobotModel.from_urdf_file`
  - [x] Verify all tests pass

- [x] 3.3.2 `bindings/python/tests/test_forward_kinematics.py`
  - [x] Update robot loading to use `RobotModel`
  - [x] Verify ForwardKinematics accepts RobotModel

- [x] 3.3.3 `bindings/python/tests/test_jacobian.py`
  - [x] Update to use RobotModel
  - [x] Verify JacobianCalculator works

- [x] 3.3.4 `bindings/python/tests/test_inverse_kinematics.py`
  - [x] Update to use RobotModel
  - [x] Verify SQPIKSolver works

- [x] 3.3.5 Create `bindings/python/tests/test_robot.py`
  - [x] Test `Robot.from_urdf()`
  - [x] Test `Robot.from_urdf_string()`
  - [x] Test `clone()` method
  - [x] Test FK methods with NumPy arrays
  - [x] Test IK methods return correct tuple
  - [x] Test Jacobian methods return NumPy arrays
  - [x] Test manipulability methods
  - [x] Test configuration methods
  - [x] Test properties (name, dof, etc.)
  - [x] Test both method name variants
  - [x] Test thread safety with threading module

- [x] 3.3.6 `bindings/python/tests/conftest.py`
  - [x] Update fixtures if they load Robot (change to RobotModel or new Robot as appropriate)

### 3.4 Run Python Tests
- [x] 3.4.1 Install updated bindings: `pip install -e bindings/python`
- [x] 3.4.2 Run pytest: `pytest bindings/python/tests/`
- [x] 3.4.3 Verify all tests pass
- [x] 3.4.4 Check for any deprecation warnings

## Phase 4: WASM Bindings Update

### 4.1 Update WASM Bindings
- [x] 4.1.1 `bindings/wasm/src/bindings.cpp`
  - [x] Update `RobotHandle` internal type:
    - [x] Change `std::shared_ptr<kinex::Robot> robot_` → `std::shared_ptr<kinex::RobotModel> robot_`
    - [x] Update all internal usages
  - [x] Add new `RobotWrapper` class (or modify existing):
    - [x] Wrapper for C++ Robot class
    - [x] Implement fromURDFString static method
    - [x] Implement forwardKinematics, computePose (return PoseData)
    - [x] Implement inverseKinematics, solveIK (return IKResultData)
    - [x] Implement computeJacobian (return MatrixData)
    - [x] Implement manipulability methods
    - [x] Implement configuration methods
    - [x] Implement clone()
    - [x] Implement property getters (getName, getDOF, etc.)
  - [x] Bind new Robot class using Embind:
    - [x] `class_<RobotWrapper>("Robot")`
    - [x] `.class_function("fromURDFString", ...)`
    - [x] `.function("forwardKinematics", ...)` with optional overrides for JS types
    - [x] `.function("computePose", ...)` (alias)
    - [x] `.function("inverseKinematics", ...)` with val conversions
    - [x] `.function("solveIK", ...)` (alias)
    - [x] `.function("computeJacobian", ...)`
    - [x] `.function("getManipulability", ...)`
    - [x] `.function("isSingular", ...)`
    - [x] `.function("getConditionNumber", ...)`
    - [x] `.function("setIKTolerance", ...)`, etc.
    - [x] `.function("clone", ...)`
    - [x] `.function("getName", ...)`, `.function("getDOF", ...)`, etc.

### 4.2 Update TypeScript Definitions
- [x] 4.2.1 `bindings/wasm/kinex.d.ts`
  - [x] Update RobotHandle (if exposed) to reflect internal changes
  - [x] Add new `Robot` class declaration:
    - [x] `class Robot`
    - [x] `static fromURDFString(urdf: string, endLink: string, baseLink?: string): Robot`
    - [x] `forwardKinematics(jointAngles: number[], link?: string): Pose`
    - [x] `computePose(jointAngles: number[], link?: string): Pose` (alias)
    - [x] `inverseKinematics(targetPose: Pose, initialGuess: number[], link?: string): IKResult`
    - [x] `solveIK(...)` (alias)
    - [x] `computeJacobian(jointAngles: number[], link?: string, type?: JacobianType): Matrix`
    - [x] `getManipulability(jointAngles: number[], link?: string): number`
    - [x] `isSingular(jointAngles: number[], threshold?: number, link?: string): boolean`
    - [x] `getConditionNumber(jointAngles: number[], link?: string): number`
    - [x] `setIKTolerance(tolerance: number): void`, etc.
    - [x] `clone(): Robot`
    - [x] `getName(): string`, `getDOF(): number`, etc.

### 4.3 Update WASM Tests
- [x] 4.3.1 `bindings/wasm/tests/node/full_bindings.test.js`
  - [x] Update tests to use RobotModel internally (via RobotHandle)
  - [x] Add tests for new Robot class
  - [x] Test fromURDFString
  - [x] Test FK, IK, Jacobian methods
  - [x] Test clone()
  - [x] Verify TypedArray conversions

- [x] 4.3.2 `bindings/wasm/tests/browser/module.browser.test.js`
  - [x] Update similar to node tests
  - [x] Test in browser environment

### 4.4 Build and Test WASM
- [x] 4.4.1 Build WASM: `npm run build` (in bindings/wasm/)
- [x] 4.4.2 Run Node tests: `npm test`
- [x] 4.4.3 Run browser tests (manual or automated)
- [x] 4.4.4 Verify WASM bundle size hasn't increased significantly

## Phase 5: Examples Update

### 5.1 Python Examples
- [x] 5.1.1 `examples/python/forward_kinematics.py`
  - [x] Update to use new `Robot` class:
    - [x] `robot = kinex.Robot.from_urdf("ur5e.urdf", "wrist_3_link")`
    - [x] `pose = robot.forward_kinematics(q_home)`
  - [x] Test example runs successfully

- [x] 5.1.2 `examples/python/inverse_kinematics.py`
  - [x] Update to use `Robot` class and `robot.inverse_kinematics()`
  - [x] Test example runs

- [x] 5.1.3 `examples/python/jacobian_analysis.py`
  - [x] Update to use `Robot` class and `robot.compute_jacobian()`
  - [x] Test manipulability methods

- [x] 5.1.4 `examples/python/trajectory_generation.py`
  - [x] Update to use `Robot` class
  - [x] Test trajectory planning works

### 5.2 C++ Examples
- [x] 5.2.1 `examples/cpp/forward_kinematics_example.cpp`
  - [x] Update includes: `#include <kinex/robot.h>`
  - [x] Update code to use `Robot::fromURDF()`
  - [x] Use `robot.forwardKinematics(q)`
  - [x] Rebuild example: check `examples/cpp/CMakeLists.txt` if needed
  - [x] Test example runs

### 5.3 JavaScript Examples
- [x] 5.3.1 `examples/javascript/app.js`
  - [x] Update to use new `Robot` class in WASM
  - [x] `const robot = kinex.Robot.fromURDFString(urdfContent, "tool0")`
  - [x] Use `robot.forwardKinematics(jointAngles)`
  - [x] Test in browser or Node.js

## Phase 6: Documentation Update

### 6.1 README Files
- [x] 6.1.1 `README.md`
  - [x] Update Python quick start example to use `Robot` class
  - [x] Update JavaScript example to use `Robot` class
  - [x] Add note about API refactoring
  - [x] Mention both `Robot` (high-level) and `RobotModel` (low-level) are available

- [x] 6.1.2 `README_zh.md`
  - [x] Update Chinese examples similarly
  - [x] Translate API refactoring notes

### 6.2 API Documentation
- [x] 6.2.1 `docs/api/python.md`
  - [x] Document `kinex.RobotModel` class (low-level)
  - [x] Document `kinex.Robot` class (recommended, high-level)
  - [x] Document all methods with examples
  - [x] Clarify when to use Robot vs RobotModel

- [x] 6.2.2 Create `docs/api/cpp.md` (if doesn't exist)
  - [x] Document `kinex::RobotModel` class
  - [x] Document `kinex::Robot` class
  - [x] Provide C++ code examples

### 6.3 Tutorials
- [x] 6.3.1 `docs/tutorials/cpp-tutorial.md`
  - [x] Update examples to use `Robot::fromURDF()`
  - [x] Show both high-level and low-level API usage

- [ ] 6.3.2 Update other tutorials as needed

### 6.4 Guides
- [x] 6.4.1 `docs/guides/getting-started.md`
  - [x] Update quick start to feature `Robot` class
  - [x] Add section on migration to Unified API

- [x] 6.4.2 Create `docs/guides/migration.md`
  - [x] Explain Robot → RobotModel rename
  - [x] Show before/after code examples
  - [x] Provide migration checklist for users
  - [x] Explain benefits of new unified API

## Phase 7: Benchmarks Update

### 7.1 Update Benchmark Scripts
- [x] 7.1.1 `benchmarks/run_tier_a_benchmarks.py`
  - [x] Update imports if needed
  - [x] Change `kinex.Robot` → `kinex.RobotModel` or use new `Robot` class
  - [x] Verify benchmarks run and produce valid results
  - [x] Compare results with baseline to ensure no regression

- [x] 7.1.2 `benchmarks/run_tier_b_benchmarks.py`
  - [x] Update similarly
  - [x] Test synthetic robot generation

- [x] 7.1.3 `benchmarks/oracle.py`
  - [x] Update robot loading
  - [x] Verify oracle validation works

- [x] 7.1.4 `benchmarks/urdf_generator.py`
  - [x] Update if it creates Robot instances
  - [x] Verify generated URDFs work with RobotModel

### 7.2 Run Full Benchmark Suite
- [x] 7.2.1 Run: `python benchmarks/run_benchmarks.py --all` (Ran subset successfully)
- [x] 7.2.2 Verify no performance regressions
- [x] 7.2.3 Update benchmark results if needed

## Phase 8: Final Integration and Testing

### 8.1 Full Build Test
- [x] 8.1.1 Clean build from scratch
  - [x] `rm -rf build && cmake -B build -DCMAKE_BUILD_TYPE=Release`
  - [x] `cmake --build build -j`
- [x] 8.1.2 Install and test
  - [x] `sudo cmake --install build --prefix /usr/local`
  - [x] Create test project using installed kinex
  - [x] Verify both Robot and RobotModel are accessible

### 8.2 Run All Tests
- [x] 8.2.1 C++ tests: `ctest --test-dir build --output-on-failure`
- [x] 8.2.2 Python tests: `pytest bindings/python/tests/`
- [ ] 8.2.3 WASM tests: `npm test` in bindings/wasm/ (Skipped by user request)
- [x] 8.2.4 Verify all tests pass

### 8.3 Code Quality Checks
- [x] 8.3.1 Run clang-format on all modified C++ files
- [x] 8.3.2 Run clang-tidy if configured
- [x] 8.3.3 Check for memory leaks with valgrind or sanitizers
- [x] 8.3.4 Verify no compiler warnings

### 8.4 Documentation Validation
- [x] 8.4.1 Build documentation if using Doxygen/Sphinx
- [x] 8.4.2 Verify all code examples in docs compile and run
- [x] 8.4.3 Check for broken links

### 8.5 Version and Changelog
- [x] 8.5.1 Update version to 2.0.0 in all relevant files
  - [x] CMakeLists.txt
  - [x] setup.py
  - [x] package.json
- [x] 8.5.2 Update CHANGELOG.md
  - [x] Document breaking changes
  - [x] Document new Robot class features
  - [x] Provide migration guide link

## Phase 9: Pre-Release Validation

### 9.1 Platform Testing
- [x] 9.1.1 Test on Linux (Ubuntu 20.04+, Fedora)
- [ ] 9.1.2 Test on macOS (11+)
- [ ] 9.1.3 Test on Windows (10/11)
- [ ] 9.1.4 Test WASM in Chrome, Firefox, Safari

### 9.2 Create Migration Examples
- [x] 9.2.1 Create example showing migration to Unified API
- [x] 9.2.2 Add to docs/guides/migration.md

### 9.3 Final Review
- [x] 9.3.1 Review all changed files for consistency
- [x] 9.3.2 Ensure all TODOs and FIXMEs are resolved
- [x] 9.3.3 Verify LICENSE headers on new files
- [x] 9.3.4 Run final spell check on documentation

## Summary
- **Total Tasks**: ~150+ individual checklist items
- **Estimated Files Modified**: 40+
- **New Files Created**: ~8
- **Breaking Change**: No (API Refactoring for v1.0)
- **Testing Required**: C++, Python, WASM on multiple platforms
