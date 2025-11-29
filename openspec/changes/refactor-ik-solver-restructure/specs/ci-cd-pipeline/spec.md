## MODIFIED Requirements

### Requirement: CI MUST build with optimized Eigen and aggressive compiler flags for Release
The CI/CD build system SHALL use optimized compiler flags for Release builds on non-MSVC toolchains that enable native instruction sets (e.g., `-march=native` for GCC/Clang) and define `EIGEN_NO_DEBUG` to remove Eigen runtime checks. These optimizations SHALL be configurable and not forced for Debug builds.

#### Scenario: CMake Release builds enable Eigen optimizations on Linux/macOS
**GIVEN** a Release build on Linux or macOS
**WHEN** CMake configures the build
**THEN** the compiler flags include `-march=native` (or equivalent on the host)
**AND** `EIGEN_NO_DEBUG` macro is set for `Kinex` targets
**AND** these flags can be disabled with a `KINEX_DISABLE_AGGRESSIVE_OPTIMIZATIONS` CMake option

#### Scenario: MSVC builds use MSVC equivalent without `-march`
**GIVEN** Release build on Windows/MSVC
**WHEN** CMake configures the build
**THEN** no `-march=native` is added (unsupported by MSVC)
**AND** MSVC appropriate optimization flags are used (e.g., `/O2`)
**AND** `EIGEN_NO_DEBUG` is still set when enabled

## ADDED Requirements

### Requirement: CI MUST measure and store performance baselines
The CI/CD pipeline SHALL add optional performance measurement steps that record FK and IK runtimes for a baseline 6-DOF robot. Results SHALL be tracked in `benchmarks/` for trend analysis.

#### Scenario: CI runs performance guard rails on PRs
**GIVEN** a pull request modifies IK or FK code
**WHEN** CI runs the performance benchmarks
**THEN** the results are stored and compared against recent baselines
**AND** a warning is created if the runtime regresses more than 10%