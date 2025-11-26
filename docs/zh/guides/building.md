# 从源码编译 Kinex

本指南介绍如何在不同平台上编译 Kinex 并配置各种编译选项。

## 系统要求

### 所有平台

- **C++20 兼容编译器**:
  - GCC 10+
  - Clang 12+
  - MSVC 19.29+ (Visual Studio 2019 16.11+)
- **CMake** 3.20 或更高版本
- **Git** (用于克隆子模块)

### 可选依赖

- **Python 3.8+** - 用于 Python 绑定
- **Node.js 18+** - 用于 JavaScript 示例和测试
- **Emscripten** - 用于 WebAssembly 绑定

## 平台特定设置

### Linux (Ubuntu/Debian)

```bash
# 安装编译工具
sudo apt-get update
sudo apt-get install -y build-essential cmake git

# 可选: Python 开发头文件
sudo apt-get install -y python3-dev python3-pip

# 可选: Node.js 用于示例
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### macOS

```bash
# 安装 Xcode 命令行工具
xcode-select --install

# 安装 Homebrew (如果尚未安装)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 安装 CMake
brew install cmake

# 可选: Python (通常已预装)
brew install python@3.11

# 可选: Node.js
brew install node
```

### Windows

**选项 1: 自动化设置 (推荐)**

```powershell
# 运行设置脚本 (检查并安装依赖)
.\scripts\setup.ps1
```

**选项 2: 手动设置**

1. 安装 **Visual Studio 2019 (16.11+)** 或 **Visual Studio 2022**
   - 选择 "使用 C++ 的桌面开发" 工作负载
   - 下载地址: https://visualstudio.microsoft.com/

2. 安装 **CMake 3.20+**
   - 通过 Chocolatey: `choco install cmake`
   - 或从此处下载: https://cmake.org/download/

3. 安装 **Git for Windows**
   - 下载地址: https://git-scm.com/download/win

## 编译 C++ 库

### 基本编译

```bash
# 克隆仓库(包含子模块)
git clone --recursive https://github.com/Daoming-Chen/kinex.git
cd kinex

# 配置
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build -j

# 安装 (Linux/macOS)
sudo cmake --install build

# 安装 (Windows, 需要管理员权限)
cmake --install build --prefix "C:\Program Files\kinex"
```

### 编译选项

使用这些 CMake 选项配置编译:

```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WASM=OFF \
  -DBUILD_BENCHMARKS=OFF
```

**可用选项:**

| 选项 | 默认值 | 说明 |
|------|-------|------|
| `CMAKE_BUILD_TYPE` | `Release` | 编译类型: `Release`, `Debug`, `RelWithDebInfo` |
| `BUILD_TESTING` | `ON` | 编译单元测试 |
| `BUILD_PYTHON_BINDINGS` | `OFF` | 编译 Python 绑定 |
| `BUILD_WASM` | `OFF` | 编译 WebAssembly 绑定 |
| `BUILD_BENCHMARKS` | `OFF` | 编译性能基准测试 |

## 编译 Python 绑定

```bash
cd bindings/python

# 安装编译依赖
pip install scikit-build-core numpy pytest

# 编译并安装 (开发模式)
pip install -e .

# 或编译 wheel 用于分发
pip wheel . -w dist --no-deps
pip install dist/kinex-*.whl
```

详见 [bindings/python/README.md](../../bindings/python/README.md)。

## 编译 WebAssembly 绑定

### 安装 Emscripten

**Linux/macOS:**

```bash
# 克隆 Emscripten SDK
cd third_party
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# 安装并激活最新版本
./emsdk install latest
./emsdk activate latest

# 设置环境 (添加到 shell 配置文件以持久化)
source ./emsdk_env.sh
```

**Windows:**

```powershell
# setup.ps1 脚本可以自动安装 Emscripten
.\scripts\setup.ps1

# 或手动安装:
cd third_party
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
.\emsdk install latest
.\emsdk activate latest
.\emsdk_env.bat
```

### 编译 WASM

```bash
# 使用 Emscripten 配置
emcmake cmake -B build-wasm \
  -DBUILD_WASM=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build-wasm -j

# 输出在: build-wasm/wasm/kinex.{js,wasm}
```

或使用提供的脚本:

```bash
# Linux/macOS
./scripts/build-wasm.sh

# Windows
.\scripts\build-wasm.ps1
```

## 运行测试

### C++ 测试

```bash
# 启用测试编译
cmake -B build -DBUILD_TESTING=ON
cmake --build build

# 运行测试
ctest --test-dir build --output-on-failure

# 或运行特定测试可执行文件
./build/core/tests/kinex_tests
```

### Python 测试

```bash
cd bindings/python
pytest tests/ -v
```

### WebAssembly 测试

```bash
cd bindings/wasm

# Node.js 测试
npm run test:node

# 浏览器测试 (需要 Chrome/Chromium)
npm run test:browser
```

## 故障排除

### Windows 特定问题

**问题: CMake 找不到 MSVC**
- 打开 "Developer Command Prompt for VS 2022" 或 "x64 Native Tools Command Prompt"
- 从那里运行 CMake 命令

**问题: 长路径错误**
```powershell
# 在 Windows 注册表中启用长路径
Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
  -Name LongPathsEnabled -Value 1
```

### 常见编译问题

**问题: 缺少 Eigen 头文件**
```bash
# 确保子模块已初始化
git submodule update --init --recursive
```

**问题: 编译器不支持 C++20**
```bash
# 更新编译器
# Ubuntu: sudo apt-get install g++-10
# macOS: brew install gcc@11
```

## 下一步

- [入门指南](getting-started.md)
- [C++ 教程](../tutorials/cpp-tutorial.md)
- [Python API 参考](../api/python.md)
- [基准测试指南](../../benchmarks/README.md)
