cmake_minimum_required(VERSION 3.20)

project(kinex
    VERSION 2.0.0
    DESCRIPTION "Modern C++20 robotics kinematics library"
    LANGUAGES CXX C
)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# For MSVC, explicitly set the C++20 flag
if(MSVC)
    add_compile_options(/std:c++20)
endif()

# Check compiler version
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.0")
        message(FATAL_ERROR "GCC version must be at least 10.0 for C++20 support. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10.0")
        message(FATAL_ERROR "Clang version must be at least 10.0 for C++20 support. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.29")
        message(FATAL_ERROR "MSVC version must be at least 19.29 (VS 2019 16.11) for C++20 support. Current version: ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
endif()

# Build options
option(BUILD_TESTING "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_WASM "Build WebAssembly bindings" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_BENCHMARKS "Build benchmark suite" OFF)

# Force static libraries for WASM builds (Emscripten doesn't support shared libraries)
if(EMSCRIPTEN)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
    message(STATUS "Forcing static libraries for Emscripten/WASM build")
endif()

# Compiler flags
if(MSVC)
    add_compile_options(/W4 /WX)
    add_compile_options(/bigobj)  # For large template instantiations
    add_compile_options(/EHsc)    # Enable exception handling
    # Disable specific warnings
    add_compile_options(/wd4127)  # conditional expression is constant (from Eigen)
    add_compile_options(/wd4251)  # needs to have dll-interface (handled by export.h)
    add_compile_options(/wd4275)  # non dll-interface class used as base
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
    add_compile_options(-Wno-unused-parameter)
    add_compile_options(-Wno-empty-body)
    # Suppress warnings from Eigen AVX512 code
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options(-Wno-error=stringop-overflow)
        add_compile_options(-Wno-stringop-overflow)
    endif()
    # Enable __declspec support for Emscripten
    if(EMSCRIPTEN)
        add_compile_options(-fdeclspec)
        # Emscripten doesn't support dllimport/dllexport, ignore the attributes
        add_compile_options(-Wno-ignored-attributes)
    endif()
endif()

# Optimization flags for Release build
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        add_compile_options(/O2)
        # Disable Eigen debug checks for performance
        add_compile_definitions(EIGEN_NO_DEBUG)
    else()
        if(EMSCRIPTEN)
            add_compile_options(-O3)
            add_compile_definitions(EIGEN_NO_DEBUG)
        else()
            # Allow users to enable -march=native for maximum performance
            # Default to -march=x86-64 for better compatibility
            option(ENABLE_NATIVE_ARCH "Enable -march=native for maximum CPU-specific optimizations" OFF)
            
            if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64")
                if(ENABLE_NATIVE_ARCH)
                    message(STATUS "Enabling -march=native for CPU-specific optimizations")
                    add_compile_options(-O3 -march=native)
                else()
                    add_compile_options(-O3 -march=x86-64)
                endif()
            else()
                add_compile_options(-O3)
            endif()
            
            # Disable Eigen debug checks for performance
            add_compile_definitions(EIGEN_NO_DEBUG)
        endif()
    endif()
endif()

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include dependencies
include(cmake/Dependencies.cmake)

# Core library
add_subdirectory(core)

# Tests (now handled within core/)
# Kept here for backward compatibility with existing build scripts
# if(BUILD_TESTING)
#     enable_testing()
#     add_subdirectory(tests)
# endif()

# Enable testing for core library
if(BUILD_TESTING)
    enable_testing()
endif()

# Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples/cpp)
endif()

# Benchmarks
if(BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings/python)
endif()

# WebAssembly bindings
if(BUILD_WASM)
    add_subdirectory(bindings/wasm)
endif()

# Installation (skip for Emscripten builds to avoid exporting host-only deps)
if(NOT EMSCRIPTEN)
    include(GNUInstallDirs)

    # Installation now handled in core/CMakeLists.txt
    
    # Generate and install CMake config file
    include(CMakePackageConfigHelpers)

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/kinexConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/kinexConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinex
    )

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/kinexConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/kinexConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/kinexConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinex
    )
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "kinex ${PROJECT_VERSION} configuration summary:")
message(STATUS "  CMake version:       ${CMAKE_VERSION}")
message(STATUS "  C++ compiler:        ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Build type:          ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build shared libs:   ${BUILD_SHARED_LIBS}")
message(STATUS "  Build testing:       ${BUILD_TESTING}")
message(STATUS "  Build examples:      ${BUILD_EXAMPLES}")
message(STATUS "  Build Python:        ${BUILD_PYTHON_BINDINGS}")
message(STATUS "  Build WASM:          ${BUILD_WASM}")
message(STATUS "  Install prefix:      ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
