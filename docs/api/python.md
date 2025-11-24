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

### Robot

Represents a robot model parsed from URDF.

```python
robot = kinex.Robot.from_urdf_file("robot.urdf")
print(robot.get_name())
print(f"DOF: {robot.dof}")
```

## Kinematics

### ForwardKinematics

Computes end-effector or link poses given joint angles.

```python
fk = kinex.ForwardKinematics(robot, "tool0")
pose = fk.compute(np.zeros(6))
```

### JacobianCalculator

Computes Jacobian matrices and kinematic metrics.

```python
jac_calc = kinex.JacobianCalculator(robot, "tool0")
J = jac_calc.compute(np.zeros(6))
manip = jac_calc.get_manipulability(np.zeros(6))
```

## Inverse Kinematics

### SQPIKSolver

Solves inverse kinematics using Sequential Quadratic Programming.

```python
solver = kinex.SQPIKSolver(robot, "tool0")
result = solver.solve(target_pose, initial_guess)

if result.status.converged:
    print(result.solution)
```

