# Design: CI/CD Pipeline

## Architecture Overview

The CI/CD pipeline consists of three main workflow files:
1. **ci.yml**: Continuous Integration - builds and tests on every push/PR
2. **release.yml**: Release workflow - builds, tests, and publishes on version tags
3. (Optional) **nightly.yml**: Scheduled builds for early detection of dependency issues

## Workflow Structure

### CI Workflow (ci.yml)
**Triggers**: `push`, `pull_request` on `main` branch

```
┌─────────────────────────────────────────────────────────�?
�? Build & Test Matrix                                     �?
�? ┌────────────�? ┌────────────�? ┌────────────�?       �?
�? �? Ubuntu    �? �?  macOS    �? �? Windows   �?       �?
�? �? 20.04     �? �?  latest   �? �? latest    �?       �?
�? └──────┬─────�? └──────┬─────�? └──────┬─────�?       �?
�?        �?               �?               �?             �?
�?   ┌────▼────�?     ┌───▼────�?     ┌───▼────�?       �?
�?   �?C++ Build�?     �?C++ Build�?   �?C++ Build�?     �?
�?   �?& Test   �?     �?& Test   �?   �?& Test   �?     �?
�?   └────┬─────�?     └────┬────�?   └────┬────�?       �?
�?        �?                �?               �?            �?
�?   ┌────▼────�?     ┌────▼────�?     ┌───▼────�?      �?
�?   │Python   �?     │Python   �?     │Python  �?       �?
�?   │Build &  �?     │Build &  �?     │Build & �?       �?
�?   │Test     �?     │Test     �?     │Test    �?       �?
�?   └────┬────�?     └────┬────�?     └────┬───�?       �?
�?        �?               �?                �?            �?
�?   ┌────▼────�?     ┌───▼────�?      ┌───▼────�?      �?
�?   │WASM     �?     │WASM    �?      │WASM    �?       �?
�?   │Build &  �?     │Build & �?      │Build & �?       �?
�?   │Test     �?     │Test    �?      │Test    �?       �?
�?   └─────────�?     └────────�?      └────────�?       �?
└─────────────────────────────────────────────────────────�?
```

### Release Workflow (release.yml)
**Triggers**: `push` with tags matching `v*.*.*`

```
┌──────────────────────────────────────────────────────────�?
�? Build Artifacts (Matrix)                                 �?
�? ┌────────────�? ┌────────────�? ┌────────────�?        �?
�? �? Ubuntu    �? �?  macOS    �? �? Windows   �?        �?
�? └──────┬─────�? └──────┬─────�? └──────┬─────�?        �?
�?        �?               �?               �?              �?
�?   ┌────▼───────────────▼────────────────▼────�?         �?
�?   �? Build Python Wheels (cibuildwheel)      �?         �?
�?   �? - Linux: manylinux2014                  �?         �?
�?   �? - macOS: universal2                     �?         �?
�?   �? - Windows: AMD64                        �?         �?
�?   └────┬─────────────────────────────────────�?         �?
�?        �?                                                �?
�?   ┌────▼────�?                                          �?
�?   │Build WASM�?                                         �?
�?   │Package   �?                                         �?
�?   └────┬─────�?                                         �?
�?        �?                                                �?
�? ┌──────▼───────────────────────────────────────�?      �?
�? �? Test All Artifacts                           �?      �?
�? �? ┌─────────�? ┌─────────�? ┌─────────�?     �?      �?
�? �? │C++ Tests�? │Py Tests �? │JS Tests �?     �?      �?
�? �? └─────────�? └─────────�? └─────────�?     �?      �?
�? └──────┬───────────────────────────────────────�?      �?
�?        �?All tests passed?                              �?
�?        �?                                                �?
�? ┌──────▼───────────────────────────────────────�?      �?
�? �? Publish                                      �?      �?
�? �? ┌─────────────�?   ┌─────────────�?        �?      �?
�? �? │PyPI Upload  �?   │npm publish  �?        �?      �?
�? �? �?twine)      �?   �?            �?        �?      �?
�? �? └─────────────�?   └─────────────�?        �?      �?
�? └──────────────────────────────────────────────�?      �?
└──────────────────────────────────────────────────────────�?
```

## Technical Decisions

### 1. Build Matrix Strategy
**Decision**: Use GitHub Actions matrix strategy for parallel builds

**Rationale**:
- Faster feedback (parallel execution)
- Consistent configuration across platforms
- Easy to add new platforms or Python versions

**Configuration**:
```yaml
strategy:
  matrix:
    os: [ubuntu-20.04, macos-latest, windows-latest]
    python-version: ['3.8', '3.9', '3.10', '3.11', '3.12']
```

### 2. Python Wheel Building
**Decision**: Use `cibuildwheel` for building Python wheels

**Rationale**:
- Handles manylinux compliance automatically
- Supports all platforms (Linux, macOS, Windows)
- Tests wheels in isolated environments
- Industry standard for compiled Python extensions

**Alternative Considered**: Direct `pip wheel` - rejected because it doesn't handle manylinux properly

### 3. Dependency Caching
**Decision**: Cache CMake build directories, pip packages, and npm packages

**Rationale**:
- Significantly speeds up workflow runs (2-5x faster)
- Reduces GitHub Actions minutes consumption
- Uses built-in `actions/cache` action

**Cache Keys**:
- CMake: `${{ runner.os }}-cmake-${{ hashFiles('**/CMakeLists.txt') }}`
- pip: `${{ runner.os }}-pip-${{ hashFiles('**/pyproject.toml') }}`
- npm: `${{ runner.os }}-npm-${{ hashFiles('**/package-lock.json') }}`

### 4. WASM Building
**Decision**: Build WASM on Ubuntu with Emscripten from emsdk

**Rationale**:
- WASM builds are platform-independent (output is the same)
- Ubuntu provides fastest build times on GitHub Actions
- emsdk installation is straightforward on Linux

**Process**:
```bash
# Install emsdk
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh

# Build WASM
emcmake cmake -B build-wasm -DBUILD_WASM=ON
cmake --build build-wasm
```

### 5. Test Execution Strategy
**Decision**: Run tests on the same platform where artifacts are built

**Rationale**:
- Ensures artifacts work on target platform
- Faster than downloading artifacts and testing separately
- Catches platform-specific runtime issues

**Test Commands**:
- C++: `ctest --test-dir build --output-on-failure`
- Python: `pytest bindings/python/tests`
- JavaScript: `npm test` (runs both Node and browser tests)

### 6. Publishing Strategy
**Decision**: Conditional publishing based on tag pattern and test success

**Triggers**:
- **PyPI**: Only on tags matching `v*` (e.g., `v0.1.0`)
- **npm**: Only on tags matching `v*`
- **Requirement**: All tests must pass on all platforms

**Secrets Required**:
- `PYPI_API_TOKEN`: PyPI API token for package upload
- `NPM_TOKEN`: npm authentication token

**Publishing Commands**:
```bash
# Python (using twine)
python -m pip install twine
python -m twine upload dist/*.whl

# JavaScript (using npm)
npm publish --access public
```

### 7. Security Considerations
**Decision**: Use minimal permissions and trusted actions only

**Implementation**:
- Set `permissions: read-all` by default
- Elevate to `contents: write` only for release uploads
- Use official GitHub actions (`actions/*`) and well-maintained third-party actions
- Pin action versions to specific commits (not `@latest`)

### 8. Fail-Fast vs Complete Matrix
**Decision**: Use `fail-fast: false` for matrix builds

**Rationale**:
- See failures on all platforms even if one fails
- Better for debugging platform-specific issues
- Slightly slower but more informative

## Platform-Specific Considerations

### Linux (Ubuntu 20.04)
- **Compiler**: GCC 9+ or Clang 10+
- **Python Wheels**: manylinux2014 for broad compatibility
- **Dependencies**: Install via `apt-get` (cmake, build-essential)

### macOS
- **Compiler**: AppleClang (comes with Xcode Command Line Tools)
- **Python Wheels**: universal2 (Intel + Apple Silicon)
- **Dependencies**: Install via `brew` if needed

### Windows
- **Compiler**: MSVC 2019+ (GitHub Actions includes Visual Studio)
- **Python Wheels**: AMD64 architecture
- **Special Handling**: Use PowerShell for build scripts
- **CMake Generator**: Use "Visual Studio 16 2019" or Ninja

## Workflow Artifacts

Workflows will upload artifacts for debugging and manual testing:
- **C++ Build Artifacts**: Compiled libraries and test executables
- **Python Wheels**: `.whl` files for each platform/Python version
- **WASM Artifacts**: `.wasm` and `.js` files
- **Test Results**: JUnit XML reports for all test suites

Retention: 30 days (GitHub Actions default)

## Future Enhancements

1. **Code Coverage**: Add coverage reports using codecov.io
2. **Benchmarks**: Run performance benchmarks and track over time
3. **Documentation**: Auto-generate and deploy API docs
4. **Docker Images**: Build and publish Docker images with pre-built binaries
5. **Pre-release Channel**: Publish to test.pypi.org for testing
