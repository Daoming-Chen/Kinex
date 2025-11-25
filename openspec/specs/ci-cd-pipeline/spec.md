# ci-cd-pipeline Specification

## Purpose
TBD - created by archiving change add-ci-cd-pipeline. Update Purpose after archive.
## Requirements
### Requirement: Multi-Platform Build Support
The CI/CD pipeline MUST build the kinex library on Linux, macOS, and Windows platforms.

#### Scenario: Building C++ core library on all platforms
**Given** a push or pull request to the repository  
**When** the CI workflow is triggered  
**Then** the C++ core library must be built successfully on Ubuntu 20.04, macOS (latest), and Windows (latest)  
**And** the build artifacts must include compiled libraries and test executables

#### Scenario: Building Python wheels for multiple versions
**Given** a push with a version tag (e.g., `v0.1.0`)  
**When** the release workflow is triggered  
**Then** Python wheels must be built for Python versions 3.8, 3.9, 3.10, 3.11, and 3.12  
**And** wheels must be built for each supported platform (Linux, macOS, Windows)  
**And** Linux wheels must comply with manylinux2014 standard  
**And** macOS wheels must be universal2 (Intel + Apple Silicon)

#### Scenario: Building WebAssembly packages
**Given** a push or pull request to the repository  
**When** the CI workflow is triggered  
**Then** the WASM bindings must be built using Emscripten  
**And** the build must produce `.wasm`, `.js`, and `.d.ts` files  
**And** the build must succeed on at least one platform (preferably Linux)

### Requirement: Automated Test Execution
All test suites MUST run automatically on each platform and MUST pass before allowing merges or publishes.

#### Scenario: Running C++ tests across platforms
**Given** the C++ core library has been built  
**When** the test job executes  
**Then** all GTest unit tests must run via `ctest`  
**And** tests must pass on all platforms (Ubuntu, macOS, Windows)  
**And** test failures must be reported with detailed output

#### Scenario: Running Python tests across platforms
**Given** Python wheels have been built  
**When** the test job executes  
**Then** all pytest tests must run on each platform  
**And** tests must pass for each Python version (3.8-3.12)  
**And** tests must verify forward kinematics, inverse kinematics, Jacobian, and robot model functionality

#### Scenario: Running JavaScript/WASM tests
**Given** the WASM package has been built  
**When** the test job executes  
**Then** Jest tests must run in Node.js environment  
**And** Jest tests must run in browser environment (using Puppeteer)  
**And** both test suites must pass before proceeding

### Requirement: Conditional Package Publishing
Packages MUST be published to PyPI and npm only when all tests pass and only on specific triggers.

#### Scenario: Publishing Python package to PyPI on release
**Given** a version tag (e.g., `v0.1.0`) has been pushed  
**And** all platform builds have succeeded  
**And** all tests have passed on all platforms  
**When** the publish job executes  
**Then** all Python wheels must be uploaded to PyPI using twine  
**And** the upload must use the `PYPI_API_TOKEN` secret  
**And** the published package must be installable via `pip install kinex`

#### Scenario: Publishing JavaScript package to npm on release
**Given** a version tag (e.g., `v0.1.0`) has been pushed  
**And** the WASM build has succeeded  
**And** all JavaScript tests have passed  
**When** the publish job executes  
**Then** the package must be published to npm registry  
**And** the publish must use the `NPM_TOKEN` secret  
**And** the published package must be installable via `npm install @kinex/wasm`

#### Scenario: Skipping publish on pull requests
**Given** a pull request is created or updated  
**When** the CI workflow runs  
**Then** build and test jobs must execute  
**But** publish jobs must not execute  
**And** the workflow must report success if all tests pass

### Requirement: Build Caching and Optimization
The pipeline MUST use caching to reduce build times and resource consumption.

#### Scenario: Caching CMake build dependencies
**Given** a workflow run is starting  
**When** the build job initializes  
**Then** CMake build artifacts must be restored from cache if available  
**And** the cache key must include OS and CMakeLists.txt hash  
**And** cache must be saved after successful builds for future runs

#### Scenario: Caching Python dependencies
**Given** a workflow run is starting  
**When** Python setup completes  
**Then** pip packages must be restored from cache if available  
**And** the cache key must include OS and pyproject.toml hash  
**And** commonly used packages (numpy, pytest) must be cached

#### Scenario: Caching npm dependencies
**Given** a workflow run is starting  
**When** Node.js setup completes  
**Then** npm packages must be restored from cache if available  
**And** the cache key must include OS and package-lock.json hash

### Requirement: Workflow Triggers and Branches
Workflows MUST run on appropriate events to balance feedback speed and resource usage.

#### Scenario: Running CI on pull requests
**Given** a pull request is opened, updated, or synchronized  
**When** changes affect code, tests, or build files  
**Then** the CI workflow must trigger automatically  
**And** run all build and test jobs  
**And** report status checks back to the pull request

#### Scenario: Running CI on main branch pushes
**Given** commits are pushed directly to the main branch  
**When** the push is completed  
**Then** the CI workflow must trigger automatically  
**And** run all build and test jobs  
**But** must not publish packages

#### Scenario: Running release workflow on version tags
**Given** a tag matching pattern `v*.*.*` is pushed  
**When** the tag push is completed  
**Then** the release workflow must trigger automatically  
**And** run build, test, and publish jobs  
**And** publish packages only if all tests pass

### Requirement: Error Reporting and Debugging
Failed workflows MUST provide clear information about what went wrong and where.

#### Scenario: Reporting build failures
**Given** a build step fails on any platform  
**When** the workflow completes  
**Then** the workflow summary must highlight which platform failed  
**And** the log must include compiler errors or warnings  
**And** build artifacts (if any) must be uploaded for debugging

#### Scenario: Reporting test failures
**Given** tests fail on any platform  
**When** the test job completes  
**Then** the workflow must report which tests failed  
**And** test output must be shown in the logs  
**And** JUnit XML reports must be generated (if applicable)  
**And** the workflow status must be marked as failed

#### Scenario: Continuing matrix builds on partial failures
**Given** a matrix build is running across multiple platforms  
**When** one platform's build or tests fail  
**Then** the workflow must continue running on other platforms  
**And** report all failures together at the end  
**And** use `fail-fast: false` strategy

### Requirement: Security and Secrets Management
The pipeline MUST handle credentials securely and follow least-privilege principles.

#### Scenario: Using PyPI token securely
**Given** the publish job needs to upload to PyPI  
**When** authentication is required  
**Then** the workflow must use `PYPI_API_TOKEN` from GitHub secrets  
**And** the token must never be logged or exposed  
**And** the token must only be accessible to publish jobs

#### Scenario: Using npm token securely
**Given** the publish job needs to upload to npm  
**When** authentication is required  
**Then** the workflow must use `NPM_TOKEN` from GitHub secrets  
**And** configure `.npmrc` with the token  
**And** the token must never be logged or exposed

#### Scenario: Minimal workflow permissions
**Given** any workflow job is running  
**When** accessing repository resources  
**Then** the workflow must use minimal required permissions  
**And** default to `permissions: read-all`  
**And** elevate to `contents: write` only for release uploads

### Requirement: Cross-Platform Compatibility
Build scripts and commands MUST work reliably on Linux, macOS, and Windows.

#### Scenario: Running CMake builds on Windows
**Given** a Windows runner is executing the build  
**When** CMake configuration runs  
**Then** it must use appropriate generator (Visual Studio or Ninja)  
**And** handle Windows path conventions correctly  
**And** build with MSVC compiler successfully

#### Scenario: Running CMake builds on macOS
**Given** a macOS runner is executing the build  
**When** CMake configuration runs  
**Then** it must use AppleClang compiler  
**And** build universal binaries when appropriate  
**And** handle macOS-specific linking requirements

#### Scenario: Running CMake builds on Linux
**Given** a Linux runner is executing the build  
**When** CMake configuration runs  
**Then** it must use GCC or Clang compiler  
**And** follow manylinux compatibility requirements for wheels  
**And** handle Linux-specific dependencies (via apt-get)

