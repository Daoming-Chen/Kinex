# Change: Add Python Bindings for kinex Library

## Why

Python 是科学计算和机器人研究领域最流行的语言。urdfx 需要提供完整的 Python bindings 来让 Python 用户能够使用 URDF 解析、正向运动学、Jacobian 计算和逆运动学求解功能。当�?Python bindings 尚未实现，只有一个占位符 README 文件�?

## What Changes

- 使用 nanobind 创建完整�?Python bindings，绑定所有核�?C++ 类和函数
- 实现 NumPy 数组�?Eigen 向量/矩阵的自动转�?
- 提供 Pythonic API，包括类型提示和完整的文档字符串
- 创建全面�?pytest 测试套件，验证所有绑定功�?
- 实现 Python 版本的性能基准测试，与 C++ 版本对比
- 配置 CMake 构建系统支持 Python bindings
- 编写 setup.py/pyproject.toml 用于 pip 安装
- 添加 Python 使用示例和文�?

## Impact

- **Affected specs**: `python-bindings` (新增能力)
- **Affected code**:
  - 新增 `bindings/python/src/` - nanobind 绑定代码
  - 新增 `bindings/python/tests/` - pytest 测试套件
  - 新增 `bindings/python/benchmarks/` - Python 性能测试
  - 新增 `bindings/python/CMakeLists.txt` - 构建配置
  - 新增 `bindings/python/pyproject.toml` - Python 包配�?
  - 更新 `bindings/python/README.md` - 完整的使用文�?
  - 更新 `examples/python/` - Python 示例代码

## Dependencies

- 依赖现有�?nanobind submodule（已�?third_party/ 中）
- 需�?Python 3.8+ �?NumPy 1.20+
- 需�?pytest 用于测试

