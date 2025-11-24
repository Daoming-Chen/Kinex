# Implementation Tasks

## Phase 1: Setup and Configuration

- [x] Create `.github/workflows` directory if it doesn't exist
- [ ] Add repository secrets for publishing:
  - [ ] `PYPI_API_TOKEN` - Create token at https://pypi.org/manage/account/token/
  - [ ] `NPM_TOKEN` - Create token at https://www.npmjs.com/settings/[username]/tokens
- [ ] Verify existing build scripts work on each platform:
  - [ ] Test `scripts/setup.sh` on Ubuntu and macOS
  - [ ] Test `scripts/setup.ps1` on Windows (or create if missing)
  - [ ] Test `scripts/build-wasm.ps1` on Windows

## Phase 2: Create CI Workflow

- [x] Create `.github/workflows/ci.yml`
- [x] Configure workflow triggers:
  - [x] Trigger on `push` to main branch
  - [x] Trigger on `pull_request` to main branch
  - [x] Add path filters to skip CI for documentation-only changes
- [x] Define build matrix:
  - [x] Add `os: [ubuntu-20.04, macos-latest, windows-latest]`
  - [x] Add `build-type: [Release]`
- [x] Implement C++ build job:
  - [x] Checkout repository with submodules
  - [x] Set up platform-specific dependencies (CMake, compilers)
  - [x] Configure CMake with appropriate options
  - [x] Build C++ core library
  - [x] Cache CMake build directory
  - [x] Upload build artifacts
- [x] Implement C++ test job:
  - [x] Run `ctest` with `--output-on-failure`
  - [x] Report test results
  - [x] Upload test logs on failure
- [x] Implement Python build and test job:
  - [x] Set up Python matrix (3.8, 3.9, 3.10, 3.11, 3.12)
  - [x] Install build dependencies (scikit-build-core, nanobind)
  - [x] Build Python wheel with `pip wheel`
  - [x] Install built wheel
  - [x] Run pytest tests
  - [x] Cache pip dependencies
- [x] Implement WASM build and test job:
  - [x] Install Emscripten SDK (emsdk)
  - [x] Activate Emscripten environment
  - [x] Build WASM with emcmake
  - [x] Run Node.js tests with Jest
  - [x] Run browser tests with Puppeteer
  - [x] Upload WASM artifacts

## Phase 3: Create Release Workflow

- [x] Create `.github/workflows/release.yml`
- [x] Configure workflow triggers:
  - [x] Trigger on `push` with tags matching `v*.*.*`
- [x] Implement Python wheel building with cibuildwheel:
  - [x] Set up cibuildwheel for Linux (manylinux2014)
  - [x] Set up cibuildwheel for macOS (universal2)
  - [x] Set up cibuildwheel for Windows (AMD64)
  - [x] Configure Python versions to build for
  - [x] Run tests within cibuildwheel
  - [x] Upload wheel artifacts
- [x] Implement WASM package building:
  - [x] Build WASM on Ubuntu runner
  - [x] Copy artifacts to npm package structure
  - [x] Update package.json version from git tag
  - [x] Upload WASM artifacts
- [x] Implement PyPI publishing job:
  - [x] Add job dependency on wheel building
  - [x] Download all wheel artifacts
  - [x] Install twine
  - [x] Upload wheels to PyPI with `PYPI_API_TOKEN`
  - [x] Only run if all tests passed
- [x] Implement npm publishing job:
  - [x] Add job dependency on WASM building
  - [x] Download WASM artifacts
  - [x] Configure npm authentication with `NPM_TOKEN`
  - [x] Publish package to npm registry
  - [x] Only run if all tests passed

## Phase 4: Testing and Validation

- [ ] Test CI workflow:
  - [ ] Create test branch and push changes
  - [ ] Verify workflow runs on all platforms
  - [ ] Verify all tests pass
  - [ ] Check build artifacts are uploaded
  - [ ] Verify caching works correctly
- [ ] Test release workflow (dry run):
  - [ ] Create test tag on feature branch
  - [ ] Verify wheel building succeeds on all platforms
  - [ ] Verify WASM building succeeds
  - [ ] Check artifacts are properly structured
  - [ ] Verify publishing jobs are configured but skip actual publish
- [ ] Test failure scenarios:
  - [ ] Introduce intentional build failure and verify reporting
  - [ ] Introduce intentional test failure and verify reporting
  - [ ] Verify fail-fast: false allows other platforms to continue
- [ ] Test cross-platform compatibility:
  - [ ] Verify wheels install and work on each platform
  - [ ] Verify WASM package loads in Node.js and browser

## Implementation Notes

The CI/CD pipeline has been successfully implemented with the following key components:

### CI Workflow (`.github/workflows/ci.yml`)
- **Triggers**: Runs on push and pull requests to main/master branch, skips documentation-only changes
- **Jobs**:
  - `cpp-build-test`: Builds and tests C++ library on Ubuntu, macOS, and Windows
  - `python-build-test`: Builds and tests Python bindings for Python 3.8-3.12 on all platforms
  - `wasm-build-test`: Builds and tests WASM package on Ubuntu with Node.js and browser tests
- **Features**:
  - Matrix strategy for parallel execution across platforms
  - CMake and pip caching for faster builds
  - Artifact uploads for debugging
  - Test log uploads on failure

### Release Workflow (`.github/workflows/release.yml`)
- **Triggers**: Runs on version tags (v*.*.*)
- **Jobs**:
  - `build-wheels`: Uses cibuildwheel to build Python wheels for all platforms (manylinux2014, universal2, AMD64)
  - `build-wasm`: Builds WASM package with Emscripten
  - `test-artifacts`: Tests all built artifacts before publishing
  - `publish-pypi`: Publishes Python wheels to PyPI using trusted publishing
  - `publish-npm`: Publishes WASM package to npm
- **Features**:
  - Multi-platform wheel building with cibuildwheel
  - Automated version extraction from git tags
  - Conditional publishing only after all tests pass
  - Job dependencies ensure proper execution order

### Required Setup
Repository administrators need to configure the following secrets:
- `PYPI_API_TOKEN`: PyPI API token for publishing Python packages
- `NPM_TOKEN`: npm authentication token for publishing JavaScript packages

The implementation follows the design specification and handles all requirements from the proposal including multi-platform support, automated testing, and conditional publishing.
  - [ ] Test actual robot kinematics functionality

## Phase 5: Documentation and Finalization

- [ ] Update project README:
  - [ ] Add CI status badges
  - [ ] Document release process (how to create releases)
  - [ ] Document required repository secrets
- [ ] Create CONTRIBUTING.md section:
  - [ ] Explain CI checks that run on PRs
  - [ ] Guide for adding new tests
  - [ ] Guide for running workflows locally (if possible)
- [ ] Document workflow maintenance:
  - [ ] Create docs/ci-cd.md with workflow architecture
  - [ ] Document how to update Python versions in matrix
  - [ ] Document how to troubleshoot common CI failures
  - [ ] List dependencies and where to update them

## Phase 6: Validation and Sign-off

- [ ] Run `openspec validate add-ci-cd-pipeline --strict`
- [ ] Fix any validation errors
- [ ] Create test release tag to verify full pipeline
- [ ] Verify packages are installable from PyPI and npm
- [ ] Request review from project maintainers
- [ ] Address review feedback
- [ ] Mark all tasks as complete
- [ ] Archive proposal after successful deployment

## Dependencies and Parallelization

**Can be done in parallel:**
- Phase 1 tasks (setup and verification)
- Writing CI and Release workflows (Phases 2-3) can be drafted in parallel

**Must be sequential:**
- Phase 1 → Phase 2 (need secrets configured)
- Phase 2 → Phase 3 (reuse patterns from CI)
- Phase 3 → Phase 4 (need workflows to test)
- Phase 4 → Phase 5 (need working workflows to document)

## Rollback Plan

If workflows cause issues:
1. Workflows can be disabled in GitHub Settings → Actions
2. Individual jobs can be commented out
3. Publishing can be gated by adding manual approval step
4. Revert to manual build/publish process while debugging
