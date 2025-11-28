# Getting Started with Kinex

This guide will help you get started with Kinex for robotics kinematics in C++, Python, or JavaScript/WebAssembly.

## Installation

### Python (Recommended for Quick Start)

The easiest way to get started is with the Python package:

```bash
pip install kinex
```

### JavaScript/TypeScript

For web applications or Node.js:

```bash
npm install @daoming.chen/kinex
```

### C++ from Source

For native C++ development, see the [Building Guide](building.md).

```bash
git clone --recursive https://github.com/Daoming-Chen/kinex.git
cd kinex
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
sudo cmake --install build
```

## Quick Start Examples

> 💡 Recommendation: For most use cases, prefer the unified `Robot` API (create a `Robot` instance
> and call `robot.forwardKinematics`, `robot.inverseKinematics`, etc.). Low-level classes `ForwardKinematics`
> and `SQPIKSolver` are available for advanced scenarios.

### Python

```python
import kinex
import numpy as np

# Load robot from URDF file
# Specify the end-effector link name
robot = kinex.Robot.from_urdf("path/to/robot.urdf", "tool0")

# Get robot information
print(f"Robot: {robot.name}")
print(f"DOF: {robot.dof}")

# Forward Kinematics
joint_angles = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
pose = robot.forward_kinematics(joint_angles)

print(f"End-effector position: {pose.translation()}")
print(f"End-effector rotation:\n{pose.rotation()}")

# Inverse Kinematics
# Define target pose
target_pose = kinex.Transform.from_position_rpy(
    [0.4, 0.2, 0.5],  # x, y, z
    [0, 3.14, 0]      # roll, pitch, yaw
)

# Solve IK from zero initial guess
solution, status = robot.inverse_kinematics(target_pose, np.zeros(robot.dof))

if status.converged:
    print(f"IK Solution: {solution}")
    print(f"Iterations: {status.iterations}")
else:
    print("IK did not converge")

# Verify solution with FK
verification_pose = robot.forward_kinematics(solution)
position_error = np.linalg.norm(
    verification_pose.translation() - target_pose.translation()
)
print(f"Position error: {position_error * 1000:.2f} mm")
```

### JavaScript/TypeScript

```javascript
import createKinexModule from '@kinex/wasm';

async function main() {
  // Initialize WASM module
  const kinex = await createKinexModule();

  // Load URDF from string or fetch from file
  const urdfResponse = await fetch('path/to/robot.urdf');
  const urdfContent = await urdfResponse.text();

  // Create robot instance with end-effector
  const robot = kinex.Robot.fromURDFString(urdfContent, "tool0");
  console.log(`Robot: ${robot.getName()}`);
  console.log(`DOF: ${robot.getDOF()}`);

  // Forward Kinematics
  const q = new Float64Array([0, -1.57, 1.57, 0, 1.57, 0]);
  const pose = robot.forwardKinematics(q);
  console.log("Position:", pose.translation);

  // Inverse Kinematics
  const target = pose; // Use FK result as target
  const qInit = new Float64Array(robot.getDOF()).fill(0);
  
  const result = robot.inverseKinematics(target, qInit);
  
  if (result.status.converged) {
    console.log("Solution:", result.solution);
  } else {
    console.log("Failed to converge");
  }
  
  // Clean up C++ objects
  robot.delete();
}
main();
```
// Note: The unified `Robot` class provides `forwardKinematics` and `inverseKinematics` which are
// recommended for most applications. The low-level helpers (ForwardKinematics, SQPIKSolver) remain
// available for advanced use cases where you need direct control over solver configuration.
```

### C++

```cpp
#include <kinex/urdf_parser.h>
#include <kinex/kinematics.h>
#include <kinex/inverse_kinematics.h>
#include <iostream>

int main() {
    // Parse URDF file
    kinex::URDFParser parser;
    auto robot = parser.parseFile("path/to/robot.urdf");

    std::cout << "Robot: " << robot.getName() << std::endl;
    std::cout << "DOF: " << robot.getDOF() << std::endl;

    // Forward Kinematics
    kinex::ForwardKinematics fk(robot, "tool0");
    Eigen::VectorXd joint_angles(6);
    joint_angles << 0.0, -1.57, 1.57, 0.0, 1.57, 0.0;

    auto pose = fk.compute(joint_angles);
    std::cout << "Position: " << pose.translation().transpose() << std::endl;

    // Inverse Kinematics
    kinex::SQPIKSolver ik_solver(robot, "tool0");

    // Configure solver
    auto config = ik_solver.getConfig();
    config.max_iterations = 100;
    config.tolerance = 1e-6;
    ik_solver.setConfig(config);

    // Define target pose
    kinex::Transform target_pose = kinex::Transform::Identity();
    target_pose.translation() << 0.4, 0.2, 0.5;

    // Solve IK
    Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd solution;
    auto status = ik_solver.solve(target_pose, initial_guess, solution);

    if (status.converged) {
        std::cout << "Solution: " << solution.transpose() << std::endl;
        std::cout << "Iterations: " << status.iterations << std::endl;
    } else {
        std::cout << "IK did not converge" << std::endl;
    }

    return 0;
}
```

## Core Concepts

### Robot Model

Kinex uses the Unified Robot Description Format (URDF) to define robot structures:

- **Links**: Rigid bodies connected by joints
- **Joints**: Revolute (rotational) or prismatic (linear) connections
- **Kinematic Chain**: Series of links and joints from base to end-effector

### Forward Kinematics (FK)

Computes the end-effector pose given joint angles:

```
FK: joint_angles → end_effector_pose
```

**Use cases:**
- Visualizing robot configuration
- Computing workspace
- Verifying IK solutions

### Inverse Kinematics (IK)

Computes joint angles to achieve a desired end-effector pose:

```
IK: target_pose → joint_angles
```

**Features:**
- SQP-based optimization
- Joint limit constraints
- Multiple initialization strategies
- Warm-starting for trajectories

### Jacobian Matrix

The Jacobian relates joint velocities to end-effector velocities:

```
v_ee = J(q) * q_dot
```

Kinex uses analytical Jacobian computation for high performance.

## Common Patterns

### Warm-Starting IK for Trajectories

When solving IK for a sequence of poses, use the previous solution as initial guess:

```python
# Python example
previous_solution = np.zeros(robot.dof)

for target_pose in trajectory:
    result = ik.solve(target_pose, initial_guess=previous_solution)
    if result.converged:
        previous_solution = result.solution
        # Use solution...
```

### Configuring the IK Solver

```python
# Preferred: Use Robot setters for common configuration
# e.g. set the tolerance used by the unified IK solver:
robot.set_ik_tolerance(1e-6)

# Advanced: If you need to directly configure the low-level SQPIKSolver,
# you can still instantiate it from the RobotModel and use `get_config`.
ik = kinex.SQPIKSolver(robot, "tool0")
config = ik.get_config()
config.max_iterations = 200
config.tolerance = 1e-6
config.step_size = 1.0
ik.set_config(config)
```

### Computing All Link Transforms

```python
# Get transforms for all links efficiently (recommended)
all_transforms = robot.compute_all_link_transforms(joint_angles)

for link_name, transform in all_transforms.items():
    print(f"{link_name}: {transform.translation()}" )
```

## Working with URDF Files

### Loading from File

```python
# Python
robot = kinex.Robot.from_urdf("robot.urdf")
```

```javascript
// JavaScript
const response = await fetch('robot.urdf');
const urdfText = await response.text();
const robot = kinex.Robot.fromURDFString(urdfText);
```

### Loading from String

Useful when URDF is embedded or generated:

```python
# Python
urdf_string = """
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- URDF content -->
</robot>
"""
robot = kinex.Robot.from_urdf_string(urdf_string)
```

## Performance Tips

1. **Reuse solver instances**: Create FK/IK objects once and reuse them
2. **Warm-start trajectories**: Use previous solutions as initial guesses
3. **Adjust tolerance**: Balance accuracy vs. speed with solver tolerance
4. **Use analytical Jacobian**: Kinex's analytical computation is 5-10x faster than AD

## Next Steps

- [Building from Source](building.md) - Compile Kinex yourself
- [C++ Tutorial](../tutorials/cpp-tutorial.md) - In-depth C++ examples
- [Python API Reference](../api/python.md) - Complete Python API docs
- [Benchmarks](../../benchmarks/README.md) - Performance characteristics
- [Examples](../../examples/) - More complete examples

## Troubleshooting

### Common Issues

**Import Error (Python)**
```bash
# Ensure package is installed
pip install kinex

# Check installation
python -c "import kinex; print(kinex.__version__)"
```

**URDF Parsing Errors**
- Verify URDF is valid XML
- Check that mesh files exist (if referenced)
- Ensure all joint limits are specified

**IK Not Converging**
- Try different initial guesses
- Check if target pose is reachable
- Verify joint limits aren't too restrictive
- Increase max_iterations or adjust tolerance

**WASM Loading Issues**
- Ensure both `.js` and `.wasm` files are accessible
- Check CORS settings for web applications
- Verify import map configuration

## Support

- 🐛 [Report Issues](https://github.com/Daoming-Chen/Kinex/issues)
- 💬 [Discussions](https://github.com/Daoming-Chen/Kinex/discussions)
- 📖 [Documentation](../README.md)
