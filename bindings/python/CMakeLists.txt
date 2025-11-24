cmake_minimum_required(VERSION 3.20)

project(kinex_python)

# Find Python and nanobind
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(nanobind CONFIG REQUIRED)

# Disable pedantic warnings for nanobind to avoid issues with zero-size arrays
# This must be done before nanobind_add_module to affect the nanobind-static target
if(NOT MSVC)
    add_compile_options(-Wno-pedantic)
    # Also disable zero-length-array warning for Clang
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        add_compile_options(-Wno-zero-length-array)
    endif()
endif()

# Create Python extension module
nanobind_add_module(_kinex src/bindings.cpp)

# Link to kinex library
target_link_libraries(_kinex PRIVATE kinex)

# Set output properties
set_target_properties(_kinex PROPERTIES
    OUTPUT_NAME "_kinex"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/kinex"
)

# Installation rules for scikit-build-core
install(TARGETS _kinex 
    LIBRARY DESTINATION kinex
)

# Install Python package files
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/kinex/"
    DESTINATION kinex
    FILES_MATCHING PATTERN "*.py"
)
