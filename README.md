# urdfx

A modern C++20 robotics kinematics library providing URDF parsing, forward kinematics, Jacobian computation, and inverse kinematics solving capabilities with Python and WebAssembly bindings.

## Features

- **URDF Parsing**: Parse Unified Robot Description Format (URDF) files using pugixml
- **Forward Kinematics**: Compute end-effector pose from joint angles using Eigen transformations
- **Jacobian Computation**: Analytical Jacobian calculation using geometric methods (5-10x faster than automatic differentiation)
- **Inverse Kinematics**: SQP-based IK solver with joint limit constraints using DaQP
- **WebAssembly Support**: Browser-based robotics applications via Emscripten
- **Performance Benchmarks**: Google Benchmark-based performance testing suite
- **3D Visualization Examples**: Three.js-based examples for interactive robot visualization

## Quick Start

### C++ Example

```cpp
#include <urdfx/urdf_parser.h>
#include <urdfx/kinematics.h>
#include <urdfx/inverse_kinematics.h>

// Parse URDF file
urdfx::URDFParser parser;
auto robot = parser.parseFile("ur5e.urdf");

// Compute forward kinematics
urdfx::ForwardKinematics fk(robot, "tool0");
Eigen::VectorXd joint_angles(6);
joint_angles << 0.0, -1.57, 0.0, 0.0, 0.0, 0.0;
auto pose = fk.compute(joint_angles);

// Solve inverse kinematics
urdfx::SQPIKSolver ik_solver(robot, "tool0");
urdfx::Transform target_pose = /* ... set target pose ... */;
Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(6);
Eigen::VectorXd solution;
auto status = ik_solver.solve(target_pose, initial_guess, solution);
```

### Python Example

**Note**: Python bindings are currently under development. See `bindings/python/README.md` for status updates.

```python
# Coming soon - API subject to change
import urdfx
import numpy as np

# Load robot from URDF
robot = urdfx.Robot.from_urdf("ur5e.urdf")

# Compute forward kinematics
fk = urdfx.ForwardKinematics(robot, "tool0")
joint_angles = np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
pose = fk.compute(joint_angles)

# Solve inverse kinematics
ik = urdfx.SQPIKSolver(robot, "tool0")
target_pose = {...}  # Pose dictionary
initial_guess = np.zeros(6)
solution = ik.solve(target_pose, initial_guess)
```

### JavaScript/WebAssembly Example

```bash
# Install from npm
npm install urdfx
```

```javascript
import createUrdfxModule from 'urdfx';

// Initialize WASM module
const urdfx = await createUrdfxModule();

// Load robot from URDF string
const urdfXml = `<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Your URDF content -->
</robot>`;

const robot = urdfx.Robot.fromURDFString(urdfXml);

// Compute forward kinematics
const fk = new urdfx.ForwardKinematics(robot, "tool0");
const pose = fk.compute([0.0, -1.57, 0.0, 0.0, 0.0, 0.0]);
console.log('Position:', pose.position);
console.log('Quaternion:', pose.quaternion);

// Solve inverse kinematics
const ik = new urdfx.SQPIKSolver(robot, "tool0");
const targetPose = {
  position: [0.5, 0.0, 0.5],
  quaternion: [1.0, 0.0, 0.0, 0.0]  // w, x, y, z
};
const result = ik.solve(targetPose, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
console.log('Converged:', result.converged);
console.log('Solution:', result.solution);

// Clean up
ik.dispose();
fk.dispose();
robot.dispose();
```

## Installation

### npm Package (WebAssembly)

The easiest way to use urdfx in JavaScript/TypeScript projects:

```bash
npm install urdfx
```

Available on npm: https://www.npmjs.com/package/urdfx

### Building from Source

#### Prerequisites

- C++20 compatible compiler (GCC 10+, Clang 12+, MSVC 19.29+ / Visual Studio 2019 16.11+)
- CMake 3.20 or later
- Python 3.8+ (for Python bindings)
- Node.js 18+ (for visualization app)
- Emscripten (for WebAssembly bindings)


**Platform-Specific Requirements:**
- **Linux**: Build tools via `apt`, `yum`, or similar package manager
- **macOS**: Xcode Command Line Tools or Homebrew
- **Windows**:
    - Visual Studio 2019 (16.11+) or Visual Studio 2022 with "Desktop development with C++" workload
    - CMake 3.20+ (recommended to install via [Chocolatey](https://chocolatey.org/packages/cmake) or manually)
    - Python 3.8+ (for Python bindings)
    - Node.js 18+ (for visualization)
    - [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) (for WASM)
    - All-in-one dependency check and environment setup: `scripts/setup.ps1`

### Building from Source

#### Linux / macOS

```bash
# Clone with submodules
git clone --recursive https://github.com/Daoming-Chen/urdfx.git
cd urdfx

# Run setup script to check dependencies
./scripts/setup.sh

# Build C++ library
mkdir build && cd build
cmake ..
cmake --build .

# Install
sudo cmake --install .
```


#### Windows

```powershell
# Clone with submodules
git clone --recursive https://github.com/Daoming-Chen/urdfx.git
cd urdfx

# Run Windows setup script (checks/install dependencies: CMake, Python, Node.js, Emscripten, Visual Studio)
./scripts/setup.ps1

# Build C++ library (from PowerShell or VS Developer Command Prompt)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j

# Install (run as Administrator)
cmake --install build --prefix "C:\Program Files\urdfx"
```

**Windows Build Notes & Troubleshooting:**
- Always use `scripts/setup.ps1` to check/install all required tools and dependencies for Windows/MSVC builds.
- If CMake can't find MSVC, open a "Developer Command Prompt for VS 2022" or "x64 Native Tools Command Prompt" and retry.
- For long path issues, enable long paths in Windows:
    ```powershell
    Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name LongPathsEnabled -Value 1
    ```
- If submodules fail to clone, check proxy settings or manually run:
    ```powershell
    git submodule update --init --recursive
    ```
- If you see errors about missing DLL exports or import macros, ensure you are using the latest CMake and Visual Studio versions as required above.
- All dependencies (Eigen, pugixml, DaQP, spdlog) are built from source and tested for MSVC compatibility.
- For Emscripten/WebAssembly builds, run `setup.ps1` to install and activate the Emscripten SDK on Windows.

**Tested on:**
- Windows 10 and 11, Visual Studio 2019/2022, CMake 3.20+, Python 3.8+, Node.js 18+, Emscripten 3.1+

For more details, see `openspec/changes/add-windows-build-support/`.

### Building Python Bindings

**Note**: Python bindings are under development. See `bindings/python/README.md` for updates.

```bash
# Not yet available
# cd bindings/python
# pip install .
```

### Building WebAssembly

```bash
cd bindings/wasm
emcmake cmake -B build
cmake --build build

# Or use the build script (Windows/Linux)
./scripts/build-wasm.sh  # or build-wasm.ps1 on Windows
```

## Dependencies

### Core Dependencies (Git Submodules)
- **Eigen 3.4+**: Linear algebra library
- **pugixml**: Lightweight XML parser
- **DaQP**: Quadratic programming solver
- **Google Test**: Unit testing framework

### Binding Dependencies
- **nanobind**: Python bindings (smaller binary size than pybind11)
- **Emscripten**: WebAssembly compiler toolchain

### Visualization Dependencies
- **React**: UI framework
- **Three.js**: 3D rendering library
- **TypeScript**: Type-safe JavaScript
- **Vite**: Build tool

## Benchmarking

Run the Google Benchmark-based suite to measure IK and Jacobian performance:

```bash
# Build with benchmarks enabled
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON
cmake --build build --config Release -j

# Run IK benchmarks
./build/benchmarks/ik_benchmarks \
    --benchmark_out=benchmarks/results/ik_benchmarks_$(date +%Y%m%d).json \
    --benchmark_out_format=json

# Run Jacobian benchmarks
./build/benchmarks/jacobian_benchmarks \
    --benchmark_out=benchmarks/results/jacobian_benchmarks_$(date +%Y%m%d).json \
    --benchmark_out_format=json
```

### Benchmark Scenarios

**IK Benchmarks** cover three typical scenarios:
- **ColdStart**: Solve from zero initial guess, reflects worst-case convergence
- **WarmStart**: Use previous solution as initial guess, measures iteration reduction
- **Trajectory**: 24 consecutive target poses with warm-starting, evaluates steady-state performance

**Jacobian Benchmarks** measure analytical Jacobian computation performance for various robot configurations.

### Metrics

- `real_time`/`cpu_time`: Average time per solve in microseconds (lower is better)
- `iterations_per_solve`: Average iterations per IK solve (indicates numerical stability)
- `success_rate`: Convergence rate percentage (should be close to 100%)
- `avg_position_error_mm`: Average position error in millimeters for converged solutions
- `avg_rotation_error_deg`: Average rotation error in degrees for converged solutions

Benchmark results are saved to `benchmarks/results/` for historical comparison and analysis.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     urdfx Library Core                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ URDF Parser  │→ │   Forward    │→ │   Jacobian   │     │
│  │  (pugixml)   │  │  Kinematics  │  │ Computation  │     │
│  └──────────────┘  │   (Eigen)    │  │ (Analytical) │     │
│                     └──────────────┘  └──────────────┘     │
│                            ↓                  ↓             │
│                     ┌──────────────────────────────┐       │
│                     │   Inverse Kinematics         │       │
│                     │   (DaQP + SQP)               │       │
│                     └──────────────────────────────┘       │
└─────────────────────────────────────────────────────────────┘
                ↓                              ↓
    ┌──────────────────────┐      ┌──────────────────────┐
    │  Python Bindings     │      │  WASM Bindings       │
    │    (nanobind)        │      │   (Emscripten)       │
    └──────────────────────┘      └──────────────────────┘
                                              ↓
                                  ┌──────────────────────┐
                                  │  Visualization App   │
                                  │     (Three.js)       │
                                  └──────────────────────┘
```

## Key Algorithms

### Forward Kinematics
Computes end-effector pose from joint angles using transformation matrices:
```
T_end = T_base × T_joint1 × T_joint2 × ... × T_jointn
```

### Jacobian Computation
Uses geometric methods to compute the analytical Jacobian matrix:
- Efficient closed-form computation
- Supports both revolute and prismatic joints

### Inverse Kinematics (SQP-based)
Sequential Quadratic Programming approach:
1. Compute current pose: `FK(q)`
2. Compute Jacobian: `J(q)`
3. Solve QP: minimize `||J·Δq - (target - FK(q))||²`
4. Apply joint limits: `q_min ≤ q + Δq ≤ q_max`
5. Update: `q ← q + α·Δq` (with line search)
6. Repeat until convergence

## Performance

- **Forward Kinematics**: Sub-millisecond computation for typical 6-DOF manipulators
- **Jacobian**: <5µs analytical computation (5-10x faster than automatic differentiation)
- **Inverse Kinematics**: Convergence typically within 10-20 iterations for complex poses
- **Cold Start IK**: ~100-300µs per solve on modern CPUs
- **Warm Start IK**: ~50-150µs per solve with good initial guess
- **WebAssembly**: Near-native performance with Emscripten optimizations

For detailed benchmark results, see `benchmarks/results/`.

## Visualization Examples

The `examples/javascript/` directory includes Three.js visualization demos:
- Interactive 3D robot visualization
- URDF model loading and rendering
- Real-time forward kinematics updates

To run the visualization examples:
```bash
# Build WASM bindings first
./scripts/build-wasm.sh  # or build-wasm.ps1 on Windows

# Serve the examples directory
python3 -m http.server 8000
# Open http://localhost:8000/examples/javascript/visualization.html
```

## Examples

See the `examples/` and `core/tests/` directories for comprehensive examples:
- **C++ Examples**: `examples/cpp/forward_kinematics_example.cpp`
- **JavaScript Examples**: `examples/javascript/` - Forward kinematics and Three.js visualization
- **Unit Tests**: `core/tests/` - Extensive test coverage including:
  - UR5e robot forward kinematics
  - 6-DOF manipulator inverse kinematics
  - Jacobian computation and singularity detection
  - Trajectory generation with warm-starting

## Contributing

We follow conventional commits and require:
- Code formatted with clang-format (C++) or black (Python)
- All tests passing
- Type hints for Python code
- Documentation for public APIs

## License

[License information to be added]

## Citation

If you use urdfx in your research, please cite:
```
[Citation information to be added]
```

## Roadmap

- ✅ URDF parsing
- ✅ Forward kinematics
- ✅ Analytical Jacobian computation (5-10x faster than AD)
- ✅ Inverse kinematics with SQP solver
- ✅ WebAssembly bindings
- ✅ Google Benchmark performance suite
- ✅ Three.js visualization examples
- 🚧 Python bindings (nanobind-based)
- 🚧 Full-featured visualization web app
- 🚧 Collision detection integration (FCL)
- 🚧 Multi-solution IK solving
- 🚧 ROS2 integration
- 🚧 Dynamics computation

## Support

For questions, issues, or contributions:
- GitHub Issues: https://github.com/Daoming-Chen/urdfx/issues
- Documentation: [To be added]

## Acknowledgments

- Eigen for fast linear algebra
- DaQP for efficient QP solving
- The robotics community for URDF standardization
