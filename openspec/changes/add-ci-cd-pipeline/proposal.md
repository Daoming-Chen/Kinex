# Proposal: Add CI/CD Pipeline

## Summary
Create a comprehensive GitHub Actions workflow to automate building, testing, and publishing urdfx across multiple platforms (Linux, macOS, Windows) for C++ core library, Python wheels, and JavaScript/WebAssembly packages.

## Motivation
Currently, the project lacks automated CI/CD infrastructure. This means:
- No automated testing across platforms before merging changes
- Manual building and publishing of packages is error-prone and time-consuming
- No guarantee that code works on all supported platforms (Linux, macOS, Windows)
- Risk of publishing broken packages to PyPI and npm

Implementing a comprehensive CI/CD pipeline will:
- Catch platform-specific bugs early through automated multi-platform testing
- Ensure consistent build quality across all artifacts (C++, Python, WASM)
- Automate the release process to PyPI and npm
- Provide faster feedback to contributors

## Proposed Changes

### New Capability: CI/CD Pipeline
Create GitHub Actions workflows that:
1. **Multi-platform Build & Test**: Build C++ library, Python wheels, and JavaScript packages on Linux (Ubuntu), macOS, and Windows
2. **Automated Testing**: Run all test suites (C++, Python, JavaScript) on each platform
3. **Conditional Publishing**: Automatically publish to PyPI and npm only when all tests pass
4. **Matrix Strategy**: Use job matrices to parallelize builds across platforms and Python versions

### Key Components
- **Build Job**: Compile C++ core, Python bindings, and WASM bindings for each platform
- **Test Job**: Execute GTest (C++), pytest (Python), and Jest (JavaScript) test suites
- **Publish Jobs**: Deploy Python wheels to PyPI and JavaScript packages to npm (only on successful tests and specific triggers like tags)

### Triggers
- **Pull Requests**: Run build and test jobs to validate changes
- **Push to main/master**: Run full pipeline but skip publishing
- **Tags (v*)**: Run full pipeline including publishing to PyPI and npm

## Impact
- **Development Workflow**: Contributors get automated feedback on PRs
- **Release Process**: Maintainers can release by pushing a version tag
- **Quality Assurance**: Platform-specific issues caught automatically
- **Dependencies**: None - this is pure CI/CD infrastructure

## Alternatives Considered
1. **Manual Release Process**: Keep current manual approach - rejected due to high error rate and time cost
2. **Single Platform CI**: Only test on Linux - rejected because macOS/Windows support is a core requirement
3. **Separate Workflows**: Split into multiple workflow files - may be revisited if this becomes too complex

## Open Questions
- Should we publish pre-release versions from development branches?
- Do we need separate workflows for nightly builds vs releases?
- Should Python wheels be built for all Python versions (3.8-3.12) or a subset?

## Related Changes
None - this is the foundational CI/CD infrastructure.

## References
- GitHub Actions documentation: https://docs.github.com/en/actions
- cibuildwheel for Python wheels: https://cibuildwheel.readthedocs.io/
- Emscripten CI examples: https://emscripten.org/docs/compiling/CI.html
