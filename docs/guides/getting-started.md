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

### Python

```python
import kinex
import numpy as np

# Load robot from URDF file
robot = kinex.Robot.from_urdf("path/to/robot.urdf")

# Get robot information
print(f"Robot: {robot.name}")
print(f"DOF: {robot.dof}")

# Forward Kinematics
fk = kinex.ForwardKinematics(robot, end_link="tool0")
joint_angles = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
pose = fk.compute(joint_angles)

print(f"End-effector position: {pose.position}")
print(f"End-effector orientation (quaternion): {pose.quaternion}")

# Inverse Kinematics
ik = kinex.SQPIKSolver(robot, end_link="tool0")

# Define target pose
target_pose = {
    "position": [0.4, 0.2, 0.5],
    "quaternion": [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
}

# Solve IK from zero initial guess
result = ik.solve(target_pose, initial_guess=np.zeros(robot.dof))

if result.converged:
    print(f"IK Solution: {result.solution}")
    print(f"Iterations: {result.iterations}")
else:
    print("IK did not converge")

# Verify solution with FK
verification_pose = fk.compute(result.solution)
position_error = np.linalg.norm(
    verification_pose.position - target_pose["position"]
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

  const robot = kinex.Robot.fromURDFString(urdfContent);
  console.log(`Robot: ${robot.getName()}`);
  console.log(`DOF: ${robot.getDOF()}`);

  // Forward Kinematics
  const fk = new kinex.ForwardKinematics(robot, "tool0");
  const jointAngles = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0];
  const pose = fk.compute(jointAngles);

  console.log('Position:', pose.position);
  console.log('Quaternion:', pose.quaternion);

  // Inverse Kinematics
  const ik = new kinex.SQPIKSolver(robot, "tool0");

  const targetPose = {
    position: [0.4, 0.2, 0.5],
    quaternion: [1.0, 0.0, 0.0, 0.0]  // w, x, y, z
  };

  const initialGuess = new Array(robot.getDOF()).fill(0.0);
  const result = ik.solve(targetPose, initialGuess);

  if (result.converged) {
    console.log('Solution:', result.solution);
    console.log('Iterations:', result.iterations);
  } else {
    console.log('IK did not converge');
  }

  // Clean up
  ik.delete();
  fk.delete();
  robot.delete();
}

main();
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
# Python example
ik = kinex.SQPIKSolver(robot, "tool0")

config = ik.get_config()
config.max_iterations = 200
config.tolerance = 1e-6
config.step_size = 1.0
ik.set_config(config)
```

### Computing All Link Transforms

```python
# Get transforms for all links efficiently
fk = kinex.ForwardKinematics(robot, "tool0")
all_transforms = fk.compute_all_link_transforms(joint_angles)

for link_name, transform in all_transforms.items():
    print(f"{link_name}: {transform.position}")
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
