# Building Kinex from Source

This guide covers building Kinex on different platforms and configuring various build options.

## Prerequisites

### All Platforms

- **C++20 compatible compiler**:
  - GCC 10+
  - Clang 12+
  - MSVC 19.29+ (Visual Studio 2019 16.11+)
- **CMake** 3.20 or later
- **Git** (for cloning with submodules)

### Optional Dependencies

- **Python 3.8+** - For Python bindings
- **Node.js 18+** - For JavaScript examples and testing
- **Emscripten** - For WebAssembly bindings

## Platform-Specific Setup

### Linux (Ubuntu/Debian)

```bash
# Install build tools
sudo apt-get update
sudo apt-get install -y build-essential cmake git

# Optional: Python development headers
sudo apt-get install -y python3-dev python3-pip

# Optional: Node.js for examples
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### macOS

```bash
# Install Xcode Command Line Tools
xcode-select --install

# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install CMake
brew install cmake

# Optional: Python (usually pre-installed)
brew install python@3.11

# Optional: Node.js
brew install node
```

### Windows

**Option 1: Automated Setup (Recommended)**

```powershell
# Run the setup script (checks and installs dependencies)
.\scripts\setup.ps1
```

**Option 2: Manual Setup**

1. Install **Visual Studio 2019 (16.11+)** or **Visual Studio 2022**
   - Select "Desktop development with C++" workload
   - Download from: https://visualstudio.microsoft.com/

2. Install **CMake 3.20+**
   - Via Chocolatey: `choco install cmake`
   - Or download from: https://cmake.org/download/

3. Install **Git for Windows**
   - Download from: https://git-scm.com/download/win

4. (Optional) Install **Python 3.8+**
   - Download from: https://www.python.org/downloads/

5. (Optional) Install **Node.js 18+**
   - Download from: https://nodejs.org/

## Building the C++ Library

### Basic Build

```bash
# Clone repository with submodules
git clone --recursive https://github.com/Daoming-Chen/kinex.git
cd kinex

# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j

# Install (Linux/macOS)
sudo cmake --install build

# Install (Windows, run as Administrator)
cmake --install build --prefix "C:\Program Files\kinex"
```

### Build Options

Configure your build with these CMake options:

```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WASM=OFF \
  -DBUILD_BENCHMARKS=OFF
```

**Available Options:**

| Option | Default | Description |
|--------|---------|-------------|
| `CMAKE_BUILD_TYPE` | `Release` | Build type: `Release`, `Debug`, `RelWithDebInfo` |
| `BUILD_TESTING` | `ON` | Build unit tests |
| `BUILD_PYTHON_BINDINGS` | `OFF` | Build Python bindings |
| `BUILD_WASM` | `OFF` | Build WebAssembly bindings |
| `BUILD_BENCHMARKS` | `OFF` | Build performance benchmarks |

## Building Python Bindings

```bash
cd bindings/python

# Install build dependencies
pip install scikit-build-core numpy pytest

# Build and install (development mode)
pip install -e .

# Or build wheel for distribution
pip wheel . -w dist --no-deps
pip install dist/kinex-*.whl
```

See [bindings/python/README.md](../../bindings/python/README.md) for more details.

## Building WebAssembly Bindings

### Install Emscripten

**Linux/macOS:**

```bash
# Clone Emscripten SDK
cd third_party
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Install and activate latest version
./emsdk install latest
./emsdk activate latest

# Source environment (add to your shell profile for persistence)
source ./emsdk_env.sh
```

**Windows:**

```powershell
# The setup.ps1 script can install Emscripten automatically
.\scripts\setup.ps1

# Or install manually:
cd third_party
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
.\emsdk install latest
.\emsdk activate latest
.\emsdk_env.bat
```

### Build WASM

```bash
# Configure with Emscripten
emcmake cmake -B build-wasm \
  -DBUILD_WASM=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build-wasm -j

# Output will be in: build-wasm/wasm/kinex.{js,wasm}
```

Or use the provided script:

```bash
# Linux/macOS
./scripts/build-wasm.sh

# Windows
.\scripts\build-wasm.ps1
```

### Install WASM Package

```bash
cd bindings/wasm

# Install dependencies
npm install

# Run tests
npm test

# Build package
npm run build

# Publish to npm (requires authentication)
npm publish
```

## Building Benchmarks

```bash
# Configure with benchmarks enabled
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON

# Build
cmake --build build -j

# Run C++ benchmarks
./build/benchmarks/cpp/ik_benchmarks
./build/benchmarks/cpp/jacobian_benchmarks

# Build and install Python bindings first, then run Python benchmarks
cd benchmarks
python3 python/run_all_benchmarks.py
```

See [benchmarks/README.md](../../benchmarks/README.md) for detailed benchmark documentation.

## Running Tests

### C++ Tests

```bash
# Build with testing enabled
cmake -B build -DBUILD_TESTING=ON
cmake --build build

# Run tests
ctest --test-dir build --output-on-failure

# Or run specific test executable
./build/core/tests/kinex_tests
```

### Python Tests

```bash
cd bindings/python
pytest tests/ -v
```

### WebAssembly Tests

```bash
cd bindings/wasm

# Node.js tests
npm run test:node

# Browser tests (requires Chrome/Chromium)
npm run test:browser
```

## Troubleshooting

### Windows-Specific Issues

**Issue: CMake can't find MSVC**
- Open "Developer Command Prompt for VS 2022" or "x64 Native Tools Command Prompt"
- Run CMake commands from there

**Issue: Long path errors**
```powershell
# Enable long paths in Windows Registry
Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
  -Name LongPathsEnabled -Value 1
```

**Issue: Submodules fail to clone**
```powershell
# Check proxy settings and manually update submodules
git submodule update --init --recursive
```

### Common Build Issues

**Issue: Missing Eigen headers**
```bash
# Ensure submodules are initialized
git submodule update --init --recursive
```

**Issue: Compiler doesn't support C++20**
```bash
# Update your compiler
# Ubuntu: sudo apt-get install g++-10
# macOS: brew install gcc@11
```

**Issue: Emscripten not found**
```bash
# Source the Emscripten environment
source ./third_party/emsdk/emsdk_env.sh

# Or on Windows
.\third_party\emsdk\emsdk_env.bat
```

## Cross-Platform Notes

### Tested Configurations

**Linux:**
- Ubuntu 20.04, 22.04
- GCC 10, 11, 12
- Clang 12, 13, 14

**macOS:**
- macOS 11 (Big Sur), 12 (Monterey), 13 (Ventura)
- Apple Clang 13, 14
- GCC 11, 12 (via Homebrew)

**Windows:**
- Windows 10, 11
- Visual Studio 2019 (16.11+), 2022
- CMake 3.20+

### Dependencies

All core dependencies are included as Git submodules:
- **Eigen** 3.4+ - Linear algebra
- **pugixml** - XML parsing
- **DaQP** - Quadratic programming
- **Google Test** - Unit testing
- **Google Benchmark** - Performance testing (optional)
- **nanobind** - Python bindings (optional)

No external package manager required for C++ builds.

## Next Steps

- [Getting Started Guide](getting-started.md)
- [C++ Tutorial](../tutorials/cpp-tutorial.md)
- [Python API Reference](../api/python.md)
- [Benchmarking Guide](../../benchmarks/README.md)
