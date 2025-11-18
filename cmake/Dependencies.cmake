# Dependencies.cmake
# This module finds or builds all dependencies for urdfx

# Prefer using system-installed packages, fall back to submodules
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/third_party)

# Eigen3
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found on system, using submodule")
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/eigen EXCLUDE_FROM_ALL)
    # Eigen provides Eigen3::Eigen target
endif()
message(STATUS "Using Eigen3: ${EIGEN3_INCLUDE_DIR}")

# spdlog (must be available before any C++ code compilation)
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    message(STATUS "spdlog not found on system, using submodule")
    set(SPDLOG_BUILD_SHARED ${BUILD_SHARED_LIBS})
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/spdlog EXCLUDE_FROM_ALL)
    # spdlog provides spdlog::spdlog target
endif()
message(STATUS "Using spdlog")

# pugixml
find_package(pugixml QUIET)
if(NOT pugixml_FOUND)
    message(STATUS "pugixml not found on system, using submodule")
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/pugixml EXCLUDE_FROM_ALL)
    # Create alias target if not provided
    if(NOT TARGET pugixml::pugixml)
        add_library(pugixml::pugixml ALIAS pugixml)
    endif()
endif()
message(STATUS "Using pugixml")

# CppAD
find_package(CppAD QUIET)
if(NOT CppAD_FOUND)
    message(STATUS "CppAD not found on system, using submodule")
    set(cppad_prefix ${PROJECT_SOURCE_DIR}/third_party/CppAD)
    
    # CppAD is header-only for basic usage, so we can just add include path
    add_library(cppad_lib INTERFACE)
    target_include_directories(cppad_lib INTERFACE ${cppad_prefix}/include)
    
    # Generate configure.hpp from configure.hpp.in if it doesn't exist
    set(CPPAD_CONFIGURE_HPP ${cppad_prefix}/include/cppad/configure.hpp)
    if(NOT EXISTS ${CPPAD_CONFIGURE_HPP})
        message(STATUS "Generating cppad/configure.hpp")
        
        # Set required CMake variables for CppAD configuration
        set(cppad_link_flags_has_m32 0)
        set(compiler_has_conversion_warn 0)
        set(cppad_debug_and_release_01 0)
        set(use_cplusplus_2017_ok 1)
        set(cppad_version "20240000.0")
        set(cppad_has_adolc 0)
        set(cppad_has_colpack 0)
        set(cppad_has_eigen 0)
        set(cppad_has_ipopt 0)
        set(cppad_deprecated_01 0)
        set(cppad_boostvector 0)
        set(cppad_cppadvector 1)
        set(cppad_stdvector 0)
        set(cppad_eigenvector 0)
        set(cppad_has_gettimeofday 0)
        set(cppad_tape_addr_type "size_t")
        set(cppad_is_same_tape_addr_type_size_t 1)
        set(cppad_tape_id_type "size_t")
        set(cppad_max_num_threads 48)
        set(cppad_has_mkstemp 0)
        set(cppad_has_tmpnam_s 1)
        set(cppad_c_compiler_cmd "cl")
        set(cppad_c_compiler_gnu_flags 0)
        set(cppad_c_compiler_msvc_flags 1)
        set(cppad_is_same_unsigned_int_size_t 0)
        set(cppad_padding_block_t "")
        
        # Read and configure the template
        file(READ ${cppad_prefix}/include/cppad/configure.hpp.in CONFIGURE_CONTENT)
        string(CONFIGURE "${CONFIGURE_CONTENT}" CONFIGURED_CONTENT @ONLY)
        file(WRITE ${CPPAD_CONFIGURE_HPP} "${CONFIGURED_CONTENT}")
    endif()
    
    # Create alias target
    add_library(CppAD::CppAD ALIAS cppad_lib)
endif()
message(STATUS "Using CppAD")

# DaQP
# DaQP doesn't have standard CMake config, so we build from submodule
if(NOT TARGET daqp)
    message(STATUS "Building DaQP from submodule")
    set(daqp_prefix ${PROJECT_SOURCE_DIR}/third_party/daqp)
    
    # Check if DaQP has CMakeLists.txt
    if(EXISTS ${daqp_prefix}/CMakeLists.txt)
        # DaQP's CMakeLists.txt has hardcoded GCC flags that break MSVC
        # We'll build it manually to avoid those issues
        file(GLOB DAQP_SOURCES ${daqp_prefix}/src/*.c)
        file(GLOB DAQP_CODEGEN_SOURCES ${daqp_prefix}/codegen/*.c)
        add_library(daqp STATIC ${DAQP_SOURCES} ${DAQP_CODEGEN_SOURCES})
        target_include_directories(daqp PUBLIC 
            ${daqp_prefix}/include
            ${daqp_prefix}/codegen
        )
        set_target_properties(daqp PROPERTIES C_STANDARD 99)
        
        # Add profiling definition if enabled
        target_compile_definitions(daqp PUBLIC PROFILING)
        
        # Disable warnings-as-errors and specific warnings for third-party code
        if(MSVC)
            target_compile_options(daqp PRIVATE 
                /W3                          # Warning level 3 (less strict than /W4)
                /WX-                         # Don't treat warnings as errors
                /wd4244                      # Conversion warnings
                /wd4456                      # Declaration hides previous local declaration
                /wd4701                      # Potentially uninitialized variable
                /D_CRT_SECURE_NO_WARNINGS    # Disable unsafe function warnings
            )
        else()
            target_compile_options(daqp PRIVATE -w)  # Disable all warnings for GCC/Clang
        endif()
    else()
        # Build DaQP as a simple library if no CMake support
        file(GLOB DAQP_SOURCES ${daqp_prefix}/src/*.c)
        add_library(daqp STATIC ${DAQP_SOURCES})
        target_include_directories(daqp PUBLIC ${daqp_prefix}/include)
        set_target_properties(daqp PROPERTIES C_STANDARD 99)
    endif()
    
    # Create alias target
    if(NOT TARGET daqp::daqp)
        add_library(daqp::daqp ALIAS daqp)
    endif()
endif()
message(STATUS "Using DaQP")

# GoogleTest (only if building tests)
if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        message(STATUS "GoogleTest not found on system, using submodule")
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
        add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/googletest EXCLUDE_FROM_ALL)
        # GoogleTest provides GTest::gtest and GTest::gtest_main
    endif()
    message(STATUS "Using GoogleTest")
endif()

# Google Benchmark (only if building benchmarks)
if(BUILD_BENCHMARKS)
    find_package(benchmark QUIET)
    if(NOT benchmark_FOUND)
        message(STATUS "Google Benchmark not found, fetching via FetchContent")
        include(FetchContent)
        set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
        set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
        set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            googlebenchmark
            GIT_REPOSITORY https://github.com/google/benchmark.git
            GIT_TAG v1.8.3
        )
        FetchContent_MakeAvailable(googlebenchmark)
    endif()
    message(STATUS "Using Google Benchmark")
endif()

# nanobind (only if building Python bindings)
if(BUILD_PYTHON_BINDINGS)
    find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
    
    # nanobind doesn't have a standard CMake config, use submodule
    if(NOT TARGET nanobind)
        message(STATUS "Using nanobind from submodule")
        add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/nanobind EXCLUDE_FROM_ALL)
    endif()
    message(STATUS "Using nanobind with Python ${Python_VERSION}")
endif()

# Emscripten is handled via toolchain file, not here
if(BUILD_WASM)
    if(NOT EMSCRIPTEN)
        message(FATAL_ERROR "BUILD_WASM is ON but not building with Emscripten. Use: emcmake cmake ...")
    endif()
    message(STATUS "Building for WebAssembly with Emscripten")
endif()
