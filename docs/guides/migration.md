# Migration Guide: v1.x to v2.0

Kinex v2.0 introduces a new Unified Robot API that simplifies common workflows. This guide explains the changes and how to migrate your existing code.

## Key Changes

1. **Renaming**: The core `Robot` class has been renamed to `RobotModel` to better reflect its role as a data structure.
2. **New Unified Class**: A new `Robot` class has been introduced as a high-level facade that handles kinematics, solvers, and configuration.
3. **Lazy Initialization**: Solvers (FK, IK, Jacobian) are now created automatically when needed, rather than requiring manual instantiation.

## C++ Migration

### 1. Rename `Robot` to `RobotModel`

If you are using the low-level API or implementing custom solvers, you likely need to update your type names.

**Before:**
```cpp
#include <kinex/robot.h>

std::shared_ptr<kinex::Robot> robot = kinex::parseURDF("file.urdf");
```

**After:**
```cpp
#include <kinex/robot_model.h>

std::shared_ptr<kinex::RobotModel> model = kinex::URDFParser().parseFile("file.urdf");
```

### 2. Adopt the New `Robot` Class

For most applications, you should switch to the new `Robot` class.

**Before:**
```cpp
// Setup
auto robot = kinex::parseURDF("file.urdf");
kinex::ForwardKinematics fk(robot, "tool0");
kinex::SQPIKSolver ik(robot, "tool0");

// Usage
auto pose = fk.compute(q);
ik.solve(target, q_init, solution);
```

**After:**
```cpp
// Setup
auto robot = kinex::Robot::fromURDF("file.urdf", "tool0");

// Usage
auto pose = robot.forwardKinematics(q);
auto [solution, status] = robot.inverseKinematics(target, q_init);
```

## Python Migration

### 1. Class Renaming

`kinex.Robot` is now the high-level class. The old `Robot` class is now `kinex.RobotModel`.

**Before:**
```python
robot = kinex.Robot.from_urdf_file("file.urdf")
fk = kinex.ForwardKinematics(robot, "tool0")
```

**After (Low-Level):**
```python
model = kinex.RobotModel.from_urdf_file("file.urdf")
fk = kinex.ForwardKinematics(model, "tool0")
```

**After (High-Level - Recommended):**
```python
robot = kinex.Robot.from_urdf("file.urdf", "tool0")
pose = robot.forward_kinematics(q)
```

### 2. Method Changes

The new `Robot` class methods return values directly or as tuples, rather than modifying objects in place or returning complex result objects (where possible).

- `inverse_kinematics` returns `(solution, status)` tuple.
- `forward_kinematics` returns `Transform` object.

## JavaScript/WASM Migration

The JavaScript API mirrors the C++ changes.

**Before:**
```javascript
const robot = kinex.parseURDF(xml);
const fk = new kinex.ForwardKinematics(robot, "tool0");
```

**After:**
```javascript
const robot = kinex.Robot.fromURDFString(xml, "tool0");
const pose = robot.forwardKinematics(q);
```
