# kinex Python Bindings

Python bindings for the kinex robotics kinematics library.

## Installation

### From Source

Requirements:
- CMake 3.20+
- C++20 compiler
- Python 3.8+
- NumPy 1.20+

```bash
pip install .
```

## Usage

```python
import kinex
import numpy as np

# Load robot (unified Robot API)
robot = kinex.Robot.from_urdf("ur5.urdf", "tool0")

# Forward Kinematics (recommended)
# Use the unified `Robot` API directly for common workflows. For advanced users, low-level classes are still available.
q = np.zeros(robot.dof)
pose = robot.forward_kinematics(q)
print(f"Position: {pose.translation()}")

# Inverse Kinematics (recommended)
solution, status = robot.inverse_kinematics(pose, q_init=np.zeros(robot.dof))
if status.converged:
    print("IK Solution:", solution)
else:
    print("IK Failed:", status.message)
```

## Development

To build for development (editable install):

```bash
pip install --no-build-isolation -ve .
```

## Testing

```bash
pytest tests/
```
