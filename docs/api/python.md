# Python API Reference

## Overview

The `kinex` Python package provides bindings for the kinex C++ library, enabling high-performance robot kinematics and dynamics computations.

## Core Classes

### Transform

Represents a 3D rigid body transformation.

```python
import kinex
import numpy as np

# From position and quaternion (x, y, z, w)
t = kinex.Transform.from_position_quaternion([1, 0, 0], [0, 0, 0, 1])

# From position and RPY (roll, pitch, yaw)
t = kinex.Transform.from_position_rpy([1, 0, 0], [0, 0, 1.57])

# Accessors
pos = t.translation()
rot = t.rotation()
mat = t.as_matrix()
```

### Robot (Unified API)

The `Robot` class provides a high-level unified interface for all robot operations and is the recommended
entry point for most workflows (FK, IK, Jacobian, etc.). Use low-level classes (RobotModel, ForwardKinematics,
SQPIKSolver) only for advanced or specialized scenarios.

```python
# Initialize from URDF
robot = kinex.Robot.from_urdf("robot.urdf", "tool0")

# Get information
print(robot.name)
print(f"DOF: {robot.dof}")

# Clone for multi-threading
worker_robot = robot.clone()
```

#### Methods

- `forward_kinematics(q, link="")`: Compute FK
- `inverse_kinematics(target, q_init, link="")`: Solve IK
- `compute_jacobian(q, link="", type=Analytic)`: Compute Jacobian
- `get_manipulability(q)`: Compute manipulability index
- `is_singular(q, threshold)`: Check for singularity

### RobotModel (Low-Level)

Represents the structural model of the robot (links, joints, geometry). This was formerly named `Robot` in v1.x.

```python
model = kinex.RobotModel.from_urdf_file("robot.urdf")
print(model.get_name())
```

## Kinematics

### ForwardKinematics (Advanced)

Computes end-effector or link poses given joint angles. For common use, prefer the `Robot` unified API
which provides `forward_kinematics` as a single-line convenience method.

Recommended:
```python
# Using Robot unified API
pose = robot.forward_kinematics(np.zeros(robot.dof))
```

Advanced (low-level helper using `RobotModel`):
```python
# Takes RobotModel
fk = kinex.ForwardKinematics(model, "tool0")
pose = fk.compute(np.zeros(6))
```

### JacobianCalculator

Computes Jacobian matrices and kinematic metrics.

```python
jac_calc = kinex.JacobianCalculator(model, "tool0")
J = jac_calc.compute(np.zeros(6))
manip = jac_calc.get_manipulability(np.zeros(6))
```

## Inverse Kinematics

### SQPIKSolver (Advanced)

Solves inverse kinematics using Sequential Quadratic Programming. For most uses, prefer the `Robot`
API and `robot.inverse_kinematics(...)` which provides a high-level interface and built-in configuration
methods. `SQPIKSolver` can be instantiated directly for fine-grained control over solver options.

```python
# Recommended
solution, status = robot.inverse_kinematics(target_pose, q_init)
if status.converged:
    print(solution)

# Advanced
solver = kinex.SQPIKSolver(model, "tool0")
result = solver.solve(target_pose, initial_guess)
if result.status.converged:
    print(result.solution)
```