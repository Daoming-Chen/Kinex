# Kinex

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![PyPI version](https://img.shields.io/pypi/v/kinex.svg)](https://pypi.org/project/kinex/)
[![npm version](https://img.shields.io/npm/v/@daoming.chen/kinex.svg)](https://www.npmjs.com/package/@daoming.chen/kinex)

[English](README.md) | [文档](docs/zh/) | [示例](examples/)

现代化的 C++20 机器人运动学库,提供 Python 和 WebAssembly 绑定,支持基于浏览器的机器人应用。

## ✨ 特性

- 🚀 **高性能**: 解析式雅可比计算比自动微分快 5-10 倍
- 🎯 **逆运动学**: 基于 SQP 的求解器,支持关节限制(每次求解约 100-300µs)
- 🌐 **WebAssembly 支持**: 在浏览器中运行,性能接近原生代码
- 🐍 **Python 绑定**: 易用的 Python API,与 NumPy 集成
- 📊 **生产就绪**: 全面的性能基准测试和测试覆盖
- 🎨 **3D 可视化**: 基于 Three.js 的交互式机器人可视化示例

## 🎬 在线演示

体验交互式 UR5 机器人可视化: [**在线演示**](https://daoming-chen.github.io/Kinex/)

![UR5 可视化演示](docs/assets/demo-preview.png)

## 📦 快速安装

### Python (通过 pip)

```bash
pip install kinex
```

### JavaScript/TypeScript (通过 npm)

```bash
npm install @daoming.chen/kinex
```

## 🚀 快速开始

### Python 示例

```python
import kinex
import numpy as np

# 从 URDF 加载机器人
robot = kinex.Robot.from_urdf("ur5e.urdf")

# 正运动学
fk = kinex.ForwardKinematics(robot, "tool0")
joint_angles = np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
pose = fk.compute(joint_angles)
print(f"位置: {pose.position}")

# 逆运动学
ik = kinex.SQPIKSolver(robot, "tool0")
target_pose = {...}  # 目标位置和姿态
solution = ik.solve(target_pose, initial_guess=np.zeros(6))
print(f"关节解: {solution.solution}")
```

### JavaScript/WebAssembly 示例

```javascript
import createKinexModule from '@daoming.chen/kinex';

// 初始化 WASM 模块
const kinex = await createKinexModule();

// 从 URDF 字符串加载机器人
const robot = kinex.Robot.fromURDFString(urdfContent);

// 计算正运动学
const fk = new kinex.ForwardKinematics(robot, "tool0");
const pose = fk.compute([0.0, -1.57, 0.0, 0.0, 0.0, 0.0]);
console.log('位置:', pose.position);

// 求解逆运动学
const ik = new kinex.SQPIKSolver(robot, "tool0");
const targetPose = {
  position: [0.5, 0.0, 0.5],
  quaternion: [1.0, 0.0, 0.0, 0.0]  // w, x, y, z
};
const result = ik.solve(targetPose, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
console.log('解:', result.solution);
```

## 📊 性能

Kinex 为生产环境设计,具有出色的性能特性:

![基准测试结果](benchmarks/results/python_ik_benchmarks.png)

*真实机器人的 IK 求解器性能,显示求解时间、迭代次数和成功率。*

**关键指标:**
- ⚡ **冷启动 IK**: 每次求解约 100-300µs
- 🔥 **热启动 IK**: 每次求解约 50-150µs
- 📐 **雅可比计算**: <5µs (解析式)
- ✅ **成功率**: >99% 收敛
- 🌐 **WebAssembly**: 接近原生性能

查看[详细基准测试](docs/zh/benchmarks/)了解更多信息。

## 📖 文档

- [入门指南](docs/zh/guides/getting-started.md) - 安装和基本使用
- [Python API](docs/zh/api/python.md) - Python 绑定参考
- [C++ 教程](docs/zh/tutorials/cpp-tutorial.md) - C++ 使用示例
- [从源码编译](docs/zh/guides/building.md) - 编译说明
- [基准测试](docs/zh/benchmarks/) - 性能分析

## 🛠️ 从源码编译

```bash
# 克隆仓库(包含子模块)
git clone --recursive https://github.com/Daoming-Chen/kinex.git
cd kinex

# 编译 C++ 库
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# 安装
sudo cmake --install build
```

平台特定的说明请参见[编译指南](docs/zh/guides/building.md)。

## 🗺️ 路线图

- ✅ URDF 解析
- ✅ 正运动学
- ✅ 解析式雅可比计算
- ✅ 逆运动学 (SQP 求解器)
- ✅ WebAssembly 绑定
- ✅ 性能基准测试
- ✅ Three.js 可视化示例
- ✅ Python 绑定
- 🚧 完整的 Web 应用
- 🔜 碰撞检测 (集成 [COAL](https://github.com/coal-library/coal))
- 🔜 多解 IK (集成 [IKFlow](https://github.com/jstmn/ikflow))
- 🔜 笛卡尔路径跟踪 (参考 [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core))
- 🔜 ROS2 集成

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 🙏 致谢

- **Eigen** - 快速线性代数库
- **DaQP** - 高效二次规划求解器
- **LoIK** - 微分逆运动学的见解:
  ```bibtex
  @inproceedings{wingoLoIK2024,
    title = {{Linear-time Differential Inverse Kinematics: an Augmented Lagrangian Perspective}},
    author = {Wingo, Bruce and Sathya, Ajay and Caron, Stéphane and Hutchinson, Seth and Carpentier, Justin},
    year = {2024},
    booktitle = {Robotics: Science and Systems}
  }
  ```

## 📧 支持

- 🐛 [GitHub Issues](https://github.com/Daoming-Chen/Kinex/issues)
- 📖 [文档](docs/zh/)
- 💬 [讨论区](https://github.com/Daoming-Chen/Kinex/discussions)

---

为机器人社区用心打造 ❤️
