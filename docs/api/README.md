# API Documentation

This directory contains API documentation for all language bindings.

## C++ API Documentation

### Core Classes
- **[`Robot`](cpp.md#robot-class)** - Unified high-level interface (Recommended)
- **[`RobotModel`](cpp.md#robotmodel-class)** - Low-level robot structure (for advanced users)
- **[`URDFParser`](cpp.md#urdfparser-class)** - URDF file parsing utility

### Solvers (Low-level)
- **`ForwardKinematics`** - Forward kinematics computation
- **`JacobianCalculator`** - Jacobian matrix computation
- **`SQPIKSolver`** - Sequential Quadratic Programming IK solver

### Generated Documentation

```bash
cd docs/api/cpp
doxygen Doxyfile
```

The generated HTML documentation will be in `docs/api/cpp/html/`.

## Language Bindings

- **[Python API Reference](python.md)** - Python bindings documentation
- **[WebAssembly API](wasm.md)** - JavaScript/TypeScript bindings documentation (coming soon)

## Examples

Code snippets and examples are available in:
- **[C++ Examples](../../examples/cpp/)** - C++ usage examples
- **[Python Examples](../../examples/python/)** - Python usage examples
- **[JavaScript Examples](../../examples/javascript/)** - WebAssembly usage examples
