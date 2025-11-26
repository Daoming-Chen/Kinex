# Kinex 入门指南

本指南将帮助您在 C++、Python 或 JavaScript/WebAssembly 中开始使用 Kinex 进行机器人运动学计算。

## 安装

### Python (推荐快速入门)

最简单的入门方式是使用 Python 包:

```bash
pip install kinex
```

### JavaScript/TypeScript

用于 Web 应用或 Node.js:

```bash
npm install @kinex/wasm
```

### C++ 从源码编译

对于原生 C++ 开发,请参阅[编译指南](building.md)。

```bash
git clone --recursive https://github.com/Daoming-Chen/kinex.git
cd kinex
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
sudo cmake --install build
```

## 快速开始示例

### Python

```python
import kinex
import numpy as np

# 从 URDF 文件加载机器人
robot = kinex.Robot.from_urdf("path/to/robot.urdf")

# 获取机器人信息
print(f"机器人: {robot.name}")
print(f"自由度: {robot.dof}")

# 正运动学
fk = kinex.ForwardKinematics(robot, end_link="tool0")
joint_angles = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])
pose = fk.compute(joint_angles)

print(f"末端执行器位置: {pose.position}")
print(f"末端执行器姿态 (四元数): {pose.quaternion}")

# 逆运动学
ik = kinex.SQPIKSolver(robot, end_link="tool0")

# 定义目标位姿
target_pose = {
    "position": [0.4, 0.2, 0.5],
    "quaternion": [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
}

# 从零初始值求解 IK
result = ik.solve(target_pose, initial_guess=np.zeros(robot.dof))

if result.converged:
    print(f"IK 解: {result.solution}")
    print(f"迭代次数: {result.iterations}")
else:
    print("IK 未收敛")

# 用正运动学验证解
verification_pose = fk.compute(result.solution)
position_error = np.linalg.norm(
    verification_pose.position - target_pose["position"]
)
print(f"位置误差: {position_error * 1000:.2f} mm")
```

### JavaScript/TypeScript

```javascript
import createKinexModule from '@kinex/wasm';

async function main() {
  // 初始化 WASM 模块
  const kinex = await createKinexModule();

  // 从字符串或文件加载 URDF
  const urdfResponse = await fetch('path/to/robot.urdf');
  const urdfContent = await urdfResponse.text();

  const robot = kinex.Robot.fromURDFString(urdfContent);
  console.log(`机器人: ${robot.getName()}`);
  console.log(`自由度: ${robot.getDOF()}`);

  // 正运动学
  const fk = new kinex.ForwardKinematics(robot, "tool0");
  const jointAngles = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0];
  const pose = fk.compute(jointAngles);

  console.log('位置:', pose.position);
  console.log('四元数:', pose.quaternion);

  // 逆运动学
  const ik = new kinex.SQPIKSolver(robot, "tool0");

  const targetPose = {
    position: [0.4, 0.2, 0.5],
    quaternion: [1.0, 0.0, 0.0, 0.0]  // w, x, y, z
  };

  const initialGuess = new Array(robot.getDOF()).fill(0.0);
  const result = ik.solve(targetPose, initialGuess);

  if (result.converged) {
    console.log('解:', result.solution);
    console.log('迭代次数:', result.iterations);
  } else {
    console.log('IK 未收敛');
  }

  // 清理资源
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
    // 解析 URDF 文件
    kinex::URDFParser parser;
    auto robot = parser.parseFile("path/to/robot.urdf");

    std::cout << "机器人: " << robot.getName() << std::endl;
    std::cout << "自由度: " << robot.getDOF() << std::endl;

    // 正运动学
    kinex::ForwardKinematics fk(robot, "tool0");
    Eigen::VectorXd joint_angles(6);
    joint_angles << 0.0, -1.57, 1.57, 0.0, 1.57, 0.0;

    auto pose = fk.compute(joint_angles);
    std::cout << "位置: " << pose.translation().transpose() << std::endl;

    // 逆运动学
    kinex::SQPIKSolver ik_solver(robot, "tool0");

    // 配置求解器
    auto config = ik_solver.getConfig();
    config.max_iterations = 100;
    config.tolerance = 1e-6;
    ik_solver.setConfig(config);

    // 定义目标位姿
    kinex::Transform target_pose = kinex::Transform::Identity();
    target_pose.translation() << 0.4, 0.2, 0.5;

    // 求解 IK
    Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd solution;
    auto status = ik_solver.solve(target_pose, initial_guess, solution);

    if (status.converged) {
        std::cout << "解: " << solution.transpose() << std::endl;
        std::cout << "迭代次数: " << status.iterations << std::endl;
    } else {
        std::cout << "IK 未收敛" << std::endl;
    }

    return 0;
}
```

## 核心概念

### 机器人模型

Kinex 使用统一机器人描述格式 (URDF) 来定义机器人结构:

- **连杆 (Links)**: 通过关节连接的刚体
- **关节 (Joints)**: 旋转 (revolute) 或平移 (prismatic) 连接
- **运动学链 (Kinematic Chain)**: 从基座到末端执行器的连杆和关节序列

### 正运动学 (FK)

根据关节角度计算末端执行器位姿:

```
FK: 关节角度 → 末端执行器位姿
```

**应用场景:**
- 可视化机器人配置
- 计算工作空间
- 验证 IK 解

### 逆运动学 (IK)

计算达到目标末端执行器位姿所需的关节角度:

```
IK: 目标位姿 → 关节角度
```

**特性:**
- 基于 SQP 的优化
- 关节限制约束
- 多种初始化策略
- 轨迹热启动

### 雅可比矩阵

雅可比矩阵关联关节速度和末端执行器速度:

```
v_ee = J(q) * q_dot
```

Kinex 使用解析式雅可比计算以获得高性能。

## 常用模式

### 轨迹 IK 的热启动

当为一系列位姿求解 IK 时,使用前一个解作为初始值:

```python
# Python 示例
previous_solution = np.zeros(robot.dof)

for target_pose in trajectory:
    result = ik.solve(target_pose, initial_guess=previous_solution)
    if result.converged:
        previous_solution = result.solution
        # 使用解...
```

### 配置 IK 求解器

```python
# Python 示例
ik = kinex.SQPIKSolver(robot, "tool0")

config = ik.get_config()
config.max_iterations = 200
config.tolerance = 1e-6
config.step_size = 1.0
ik.set_config(config)
```

### 计算所有连杆变换

```python
# 高效获取所有连杆的变换
fk = kinex.ForwardKinematics(robot, "tool0")
all_transforms = fk.compute_all_link_transforms(joint_angles)

for link_name, transform in all_transforms.items():
    print(f"{link_name}: {transform.position}")
```

## 使用 URDF 文件

### 从文件加载

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

### 从字符串加载

当 URDF 是嵌入式或生成的时很有用:

```python
# Python
urdf_string = """
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- URDF 内容 -->
</robot>
"""
robot = kinex.Robot.from_urdf_string(urdf_string)
```

## 性能优化建议

1. **重用求解器实例**: 创建一次 FK/IK 对象并重复使用
2. **轨迹热启动**: 使用前一个解作为初始猜测
3. **调整容差**: 在精度和速度之间权衡求解器容差
4. **使用解析式雅可比**: Kinex 的解析计算比 AD 快 5-10 倍

## 下一步

- [从源码编译](building.md) - 自己编译 Kinex
- [C++ 教程](../tutorials/cpp-tutorial.md) - 深入的 C++ 示例
- [Python API 参考](../api/python.md) - 完整的 Python API 文档
- [基准测试](../../benchmarks/README.md) - 性能特性
- [示例](../../../examples/) - 更多完整示例

## 故障排除

### 常见问题

**导入错误 (Python)**
```bash
# 确保包已安装
pip install kinex

# 检查安装
python -c "import kinex; print(kinex.__version__)"
```

**URDF 解析错误**
- 验证 URDF 是有效的 XML
- 检查网格文件是否存在 (如果引用了)
- 确保指定了所有关节限制

**IK 不收敛**
- 尝试不同的初始猜测
- 检查目标位姿是否可达
- 验证关节限制不太严格
- 增加 max_iterations 或调整容差

**WASM 加载问题**
- 确保 `.js` 和 `.wasm` 文件都可访问
- 检查 Web 应用的 CORS 设置
- 验证 import map 配置

## 支持

- 🐛 [报告问题](https://github.com/Daoming-Chen/Kinex/issues)
- 💬 [讨论区](https://github.com/Daoming-Chen/Kinex/discussions)
- 📖 [文档](../README.md)
