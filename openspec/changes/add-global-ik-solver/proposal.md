# Change: Add Robust & Global Inverse Kinematics Solver

## Why
The current single-start SQP IK solver has ~60-80% success rate and can get stuck in local minima. Users need both (1) highly reliable single-solution IK for robotics applications and (2) comprehensive multi-solution exploration for path planning and configuration optimization.

## What Changes
- **Add `GlobalIKSolver` class** with parallel multi-start strategy using std::async
- **Add `GlobalSolverConfig`** with robust/global modes, thread control, and solution deduplication
- **Add `Robot::solveRobustIK()`** method for high-success-rate single-solution IK
- **Add `Robot::solveGlobalIK()`** method for comprehensive multi-solution exploration
- **Add solution quality metrics** and automatic best-solution selection
- **Add configuration space sampling** based on joint limits and random seeds

**BREAKING**: None - all additions are new APIs that don't affect existing functionality

## Impact
- **Affected specs**: [`inverse-kinematics`](specs/inverse-kinematics/spec.md) - adding global solver capabilities
- **Affected code**:
  - `include/kinex/global_inverse_kinematics.h` (new)
  - `include/kinex/robot.h` (new methods)
  - `src/global_inverse_kinematics.cpp` (new)
  - `src/robot.cpp` (new methods)