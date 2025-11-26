# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Comprehensive bilingual documentation (English & Chinese)
- GitHub Pages deployment for interactive JavaScript demo
- Simplified main README with quick start examples
- Detailed building and getting started guides
- Documentation index pages for easy navigation

### Changed
- Renamed npm package from `@kinex/wasm` to `@daoming.chen/kinex`
- Restructured documentation into `docs/` folder
- Simplified CI/CD workflows with reusable components

### Fixed
- Fixed Python CI test paths
- Fixed release action for wheel building
- Fixed cibuildwheel configuration

## [1.0.0] - TBD

### Added
- Initial release of Kinex kinematics library
- URDF parsing support
- Forward kinematics computation
- Analytical Jacobian computation
- SQP-based inverse kinematics solver
- Python bindings (nanobind)
- WebAssembly bindings (Emscripten)
- Three.js visualization examples
- Comprehensive benchmark suite
- Support for Linux, macOS, and Windows

### Features
- High-performance analytical Jacobian (5-10x faster than AD)
- IK solving with joint limit constraints
- Near-native WebAssembly performance
- Production-ready with >99% convergence rate

[Unreleased]: https://github.com/Daoming-Chen/Kinex/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/Daoming-Chen/Kinex/releases/tag/v1.0.0
