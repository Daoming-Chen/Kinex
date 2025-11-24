# Design Document: Multi-Language Project Restructure

**Change ID**: `restructure-project-layout`  
**Status**: Draft  
**Last Updated**: 2025-11-19

## Overview

本文档详细说�?kinex 项目重构的架构设计决策，重点解决跨语言项目的代码组织挑战�?

## Context and Goals

### Current State

kinex 是一�?C++20 机器人运动学库，提供�?
- 核心 C++ 库（URDF 解析、正�?逆向运动学、Jacobian 计算�?
- WebAssembly 绑定（通过 Emscripten + Embind�?
- 计划中的 Python 绑定（通过 nanobind�?
- Three.js 可视�?Web 应用

当前目录结构扁平化，导致�?
- 核心库代码（`src/`, `include/`）与绑定代码（`wasm/`）、应用代码（`visualization/`）在同一层级
- 示例代码混杂，缺少语言分类
- 难以独立构建、测试和发布各语言的绑�?

### Design Goals

1. **清晰的分�?*：Core �?Bindings �?Examples �?Apps
2. **独立�?*：每个语言绑定可以独立开发、测试和发布
3. **可扩展�?*：方便未来添加新语言绑定（Rust, Julia, Go 等）
4. **符合惯例**：遵循大型跨语言项目的最佳实�?
5. **向后兼容**：尽量减少对现有用户的影响（v1.0.0 前完成）

## Architecture Decisions

### AD-1: 采用 Monorepo 而非 Multi-Repo

**决策**：将所有语言绑定保留在同一个仓库中，而不是拆分为多个独立仓库�?

**理由**�?
- **版本同步简�?*：Core 库的变更可以立即反映到所有绑定中
- **CI/CD 简�?*：单一 CI pipeline 可以测试所有语言的集�?
- **开发效率高**：跨语言�?bug 修复�?feature 开发可以在一�?PR 中完�?
- **代码共享**：测试数据（�?URDF 文件）可以在所有绑定间共享

**权衡**�?
- �?仓库体积较大（但对于当前规模可接受）
- �?构建时间较长（可通过 CI 缓存优化�?
- �?更容易维护一致�?
- �?降低版本管理复杂�?

**替代方案考虑**�?
- **Multi-Repo**：每个绑定独立仓库（�?`kinex-python`, `kinex-wasm`�?
  - 优点：每个仓库更轻量，发布流程独�?
  - 缺点：版本同步困难，需�?git submodule 或复杂的 CI 编排
  - **结论**：对于当前规模（<10 万行代码）不值得

### AD-2: Core 库独立于 Bindings

**决策**：将 C++ 核心库代码移动到 `core/` 目录，与绑定层完全分离�?

**设计**�?

```
core/
├── include/kinex/        # Public API (header-only 优先)
├── src/                  # Implementation
└── tests/                # Unit tests

bindings/
├── python/               # Python 绑定（依�?core�?
└── wasm/                 # WASM 绑定（依�?core�?
```

**理由**�?
- **明确依赖方向**：Bindings 依赖 Core，反之则不成�?
- **独立测试**：Core 可以在没有任何绑定的情况下完整测�?
- **性能优化**：Core 可以使用�?C++ 优化，不受绑定层限制
- **文档生成**：Doxygen 只需扫描 `core/` 目录

**实现细节**�?
- `core/CMakeLists.txt` 定义 `kinex::core` target
- Bindings 通过 `target_link_libraries(kinex_python PRIVATE kinex::core)` 引用
- 安装规则：headers �?`include/kinex/`, library �?`lib/`

### AD-3: 按语言组织 Examples

**决策**：将示例代码按语言分类�?`examples/{cpp,python,javascript}/`�?

**当前问题**�?
- `examples/` 只有 C++ 示例
- Python �?JavaScript 用户需要自己摸�?API 用法
- 示例代码缺少构建说明

**新设�?*�?

```
examples/
├── cpp/
�?  ├── forward_kinematics.cpp       # 演示 FK 计算
�?  ├── inverse_kinematics.cpp       # 演示 IK 求解
�?  ├── jacobian_computation.cpp     # 演示 Jacobian
�?  └── CMakeLists.txt               # 独立构建配置
├── python/
�?  ├── forward_kinematics.py
�?  ├── inverse_kinematics.py
�?  ├── visualization.py             # Matplotlib 可视�?
�?  └── requirements.txt
└── javascript/
    ├── forward_kinematics.js        # Node.js 示例
    ├── inverse_kinematics.html      # 浏览器示�?
    └── package.json
```

**理由**�?
- **学习曲线降低**：每种语言的开发者只看相关示�?
- **独立运行**：每个目录都是一个可运行的最小项�?
- **文档友好**：README 可以直接引用对应语言的示�?

**实现注意事项**�?
- 每个示例都应该是自包含的（包括依赖声明）
- C++ 示例通过 CMake `find_package(kinex)` 引用已安装的�?
- Python 示例通过 `import kinex` 引用 pip 安装的包

### AD-4: 应用层独立（Apps�?

**决策**：将完整应用（如 `visualization/`）移动到 `apps/` 目录�?

**区分标准**�?
- **Library/Binding**：可以被其他项目导入和使�?
- **Application**：独立运行的完整程序，有自己的入口点�?UI

**当前问题**�?
- `visualization/` 在根目录，与库代码混在一�?
- 不清楚它是示例还是产品级应用

**新设�?*�?

```
apps/
└── visualization/         # Three.js Web 应用
    ├── src/
    ├── public/
    ├── package.json
    ├── vite.config.ts
    └── README.md          # 应用使用说明
```

**未来扩展可能�?*�?
```
apps/
├── visualization/         # Web 可视�?
├── cli/                   # 命令行工具（计划中）
└── desktop/               # Electron 桌面应用（未来）
```

**理由**�?
- **清晰的用�?*：用户立即知道这是可运行的应�?
- **独立维护**：应用有自己的发布周期和版本�?
- **避免混淆**：不会被误认为是库的一部分

### AD-5: 统一文档目录（Docs�?

**决策**：创�?`docs/` 目录集中管理所有文档，而不是分散在各个目录�?README�?

**结构**�?

```
docs/
├── api/                   # API 参考文档（自动生成�?
�?  ├── cpp/               # Doxygen 输出
�?  ├── python/            # Sphinx 输出
�?  └── javascript/        # TypeDoc 输出
├── guides/                # 用户指南（手写）
�?  ├── getting-started.md
�?  ├── urdf-parsing.md
�?  ├── kinematics.md
�?  └── inverse-kinematics.md
└── tutorials/             # 教程（手�?+ 代码�?
    ├── cpp-tutorial.md
    ├── python-tutorial.md
    └── web-tutorial.md
```

**理由**�?
- **便于查找**：所有文档在一个地�?
- **工具友好**：静态网站生成器（如 MkDocs）可以直接使�?
- **版本控制**：文档与代码同步更新

**�?README 的关�?*�?
- 根目�?`README.md`：项目概览、快速开始、链接到详细文档
- `docs/guides/`：深入的概念解释和使用指�?
- `examples/`：可运行的代码示�?

### AD-6: CMake 构建系统的分�?

**决策**：采用分层的 CMake 配置，每个子项目有独立的 CMakeLists.txt�?

**结构**�?

```
CMakeLists.txt                    # 根配置（版本、选项、子目录�?
├── core/CMakeLists.txt           # C++ 核心�?
├── bindings/
�?  ├── python/CMakeLists.txt     # Python 绑定（条件编译）
�?  └── wasm/CMakeLists.txt       # WASM 绑定（条件编译）
├── examples/
�?  └── cpp/CMakeLists.txt        # C++ 示例
└── benchmarks/CMakeLists.txt     # 性能测试
```

**关键设计**�?

1. **�?CMakeLists.txt**�?

```cmake
project(kinex VERSION 1.0.0)

option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_WASM "Build WebAssembly module" OFF)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTING "Build tests" ON)

add_subdirectory(core)

if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings/python)
endif()

if(BUILD_WASM)
    add_subdirectory(bindings/wasm)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples/cpp)
endif()
```

2. **Core CMakeLists.txt**�?

```cmake
add_library(kinex_core
    src/robot_model.cpp
    src/urdf_parser.cpp
    src/kinematics.cpp
    src/inverse_kinematics.cpp
)

target_include_directories(kinex_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_library(kinex::core ALIAS kinex_core)

install(TARGETS kinex_core EXPORT kinexTargets)
install(DIRECTORY include/ DESTINATION include)
```

3. **Bindings CMakeLists.txt**�?

```cmake
# bindings/wasm/CMakeLists.txt
target_link_libraries(kinex_wasm PRIVATE kinex::core)
```

**理由**�?
- **选择性构�?*：开发者可以只构建需要的部分
- **依赖清晰**：通过 CMake target 明确依赖关系
- **安装支持**：`cmake --install` 可以正确安装头文件和�?

### AD-7: 测试策略的分�?

**决策**：每个层级有独立的测试，但共享测试数据�?

**测试类型**�?

1. **Core Tests** (`core/tests/`)�?
   - 单元测试（GTest�?
   - 性能测试（Google Benchmark�?
   - 不依赖任何绑�?

2. **Binding Tests**�?
   - Python: `bindings/python/tests/` (pytest)
   - WASM: `bindings/wasm/tests/` (Jest)
   - 测试绑定层的正确性和性能

3. **Integration Tests** (`tests/integration/` - 未来)�?
   - 测试多个组件的协同工�?
   - 例如：URDF �?FK �?IK �?验证

**共享测试数据**�?

```
tests/fixtures/              # �?examples/models/
└── ur5/
    ├── ur5e.urdf
    └── meshes/
```

- 所有测试引用统一的测试数�?
- 避免重复存储相同�?URDF 文件

## Implementation Considerations

### Migration Strategy

**逐步迁移，保持构建始终可�?*�?

1. **Phase 1**：创建新目录，更�?CMake（不移动文件�?
2. **Phase 2**：移动核心文件（`include`, `src`, `tests`�?
3. **Phase 3**：移动绑定和应用
4. **Phase 4**：更新文档和清理

每个阶段结束后都要确保：
- �?CMake 配置成功
- �?构建成功
- �?所有测试通过

### Git History Preservation

使用 `git mv` 而不是手动删除和创建文件�?

```bash
git mv include core/include
git mv src core/src
git mv tests core/tests
```

这样 `git log --follow` 可以追踪文件历史�?

对于大规模重构，�?`.git-blame-ignore-revs` 中记录提交：

```
# Restructure project layout for multi-language support
abc123def456...
```

### External Impact

**影响分析**�?

1. **Include 路径变化**�?
   - 旧：`#include <kinex/robot_model.h>`（查�?`include/kinex/`�?
   - 新：`#include <kinex/robot_model.h>`（查�?`core/include/kinex/`�?
   - **缓解**：安装后路径不变（`/usr/local/include/kinex/`�?

2. **CMake Find Package**�?
   - 旧：`find_package(kinex)`
   - 新：`find_package(kinex)` + `target_link_libraries(... kinex::core)`
   - **缓解**：在 `kinexConfig.cmake` 中提供兼容�?alias

3. **Python Import**�?
   - 当前无影响（Python 绑定尚未实现�?
   - 未来：`import kinex` 保持不变

4. **WASM 模块**�?
   - 当前：从 `build-wasm/` 输出
   - 未来：从 `build/bindings/wasm/` 输出
   - **缓解**：更�?`visualization/` 的加载路�?

### Performance Considerations

**构建性能**�?
- 分层 CMake 可以提高增量构建速度
- 只修�?Python 绑定时，不需要重新编�?Core

**运行时性能**�?
- 无影响（目录结构不影响编译后的二进制�?

### Security Considerations

- 无新的安全风�?
- 继续使用 submodules 管理依赖（避免供应链攻击�?

## Future Extensions

### Adding New Language Bindings

当需要添加新语言（例�?Rust）时�?

1. 创建 `bindings/rust/` 目录
2. 添加 `bindings/rust/CMakeLists.txt` �?`Cargo.toml`
3. 实现绑定层（使用 `cxx` �?`rust-bindgen`�?
4. 添加 `examples/rust/` 示例
5. 更新�?`CMakeLists.txt` 添加 `BUILD_RUST_BINDINGS` 选项

### Separate Documentation Site

未来可以使用 MkDocs/Docusaurus 生成文档网站�?

```
docs/
├── mkdocs.yml            # MkDocs 配置
├── index.md              # 主页
├── api/                  # API 引用（链接到生成的文档）
├── guides/               # 手写指南
└── tutorials/            # 手写教程
```

部署�?GitHub Pages：`https://username.github.io/kinex/`

## Alternatives Revisited

### Why Not Bazel?

Bazel 提供强大�?monorepo 支持，但�?
- **学习曲线陡峭**：团队需要学习新的构建系�?
- **工具链复�?*：需要配�?C++, Python, WASM �?Bazel 规则
- **过度设计**：当前项目规模（<50k LOC）不需�?Bazel 的规�?

**何时考虑 Bazel**�?
- 项目增长�?>100k LOC
- 需要严格的依赖隔离和可重现构建
- 有专门的 DevOps 工程师维护构建系�?

### Why Not Conan for Dependencies?

Conan 可以管理 C++ 依赖，但�?
- **Submodules 已经工作良好**：Eigen、pugixml 等都�?header-only 或易于编�?
- **跨平台挑�?*：Conan �?WASM 支持上不�?Emscripten 原生工具�?
- **额外复杂�?*：需要维�?`conanfile.py` 和处理版本冲�?

**何时考虑 Conan**�?
- 依赖项数�?>10
- 需要管理多个版本的同一个库
- 团队已经熟悉 Conan 工作�?

## Risks and Mitigation

### Risk 1: 破坏现有用户的构�?

**概率**: �? 
**影响**: �?

**缓解措施**�?
- �?v1.0.0 正式发布前完成重�?
- 提供详细�?Migration Guide
- �?CMake 中提供向后兼容的 target alias

### Risk 2: CI/CD Pipeline 失败

**概率**: 高（第一次运行时�? 
**影响**: �?

**缓解措施**�?
- �?feature 分支上先测试 CI
- 逐个修复路径问题
- 使用 CI cache 加速重�?

### Risk 3: 文档链接失效

**概率**: �? 
**影响**: �?

**缓解措施**�?
- 使用 `markdown-link-check` 工具验证链接
- �?CI 中添加文档验证步�?

### Risk 4: Git Blame 信息丢失

**概率**: 低（如果使用 `git mv`�? 
**影响**: �?

**缓解措施**�?
- 始终使用 `git mv` 而不是手动移�?
- 使用 `.git-blame-ignore-revs` 记录重构提交
- 团队培训：使�?`git log --follow <file>` 查看历史

## Success Criteria

1. �?所�?C++ 测试通过�?00% pass rate�?
2. �?WASM 构建成功且大�?< 2MB
3. �?Visualization 应用正常运行
4. �?CI/CD pipeline 全部通过
5. �?文档链接无失效（0 broken links�?
6. �?Git 历史可追踪（`git log --follow` 可用�?
7. �?新贡献者能�?< 30 分钟内理解项目结�?

## References

- [CMake Best Practices for Multi-Language Projects](https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html)
- [Monorepo.tools](https://monorepo.tools/)
- [Google's Approach to Monorepos](https://research.google/pubs/pub45424/)
- [TensorFlow Repository Structure](https://github.com/tensorflow/tensorflow)
- [PyTorch Repository Structure](https://github.com/pytorch/pytorch)
