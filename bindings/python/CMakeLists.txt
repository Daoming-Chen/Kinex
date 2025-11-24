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

# Pass version information to the bindings
# Use CMAKE_PROJECT_VERSION instead of PROJECT_VERSION since we're in a subdirectory
target_compile_definitions(_kinex PRIVATE KINEX_VERSION="${CMAKE_PROJECT_VERSION}")

# Link to kinex library
target_link_libraries(_kinex PRIVATE kinex)

# Set RPATH for the extension module to find the shared library in the same directory
if(UNIX AND NOT APPLE)
    set_target_properties(_kinex PROPERTIES
        INSTALL_RPATH "$ORIGIN"
    )
elseif(APPLE)
    set_target_properties(_kinex PROPERTIES
        INSTALL_RPATH "@loader_path"
    )
endif()

# Set output properties
set_target_properties(_kinex PROPERTIES
    OUTPUT_NAME "_kinex"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/kinex"
)

# Installation rules for scikit-build-core
install(TARGETS _kinex 
    LIBRARY DESTINATION kinex
)

# Install the kinex shared library alongside the Python extension
install(TARGETS kinex
    RUNTIME DESTINATION kinex  # For Windows DLLs
    LIBRARY DESTINATION kinex  # For Unix shared libraries
)

# Install third-party shared libraries (spdlog, pugixml)
# These are needed at runtime on all platforms
if(TARGET spdlog)
    install(TARGETS spdlog
        RUNTIME DESTINATION kinex
        LIBRARY DESTINATION kinex
    )
endif()

# pugixml-shared is the actual target when BUILD_SHARED_LIBS is ON
# pugixml is just an INTERFACE library that aliases to pugixml-shared or pugixml-static
if(TARGET pugixml-shared)
    install(TARGETS pugixml-shared
        RUNTIME DESTINATION kinex
        LIBRARY DESTINATION kinex
    )
elseif(TARGET pugixml-static)
    # Static libraries don't need runtime installation, they're linked into the executable
    install(TARGETS pugixml-static
        ARCHIVE DESTINATION kinex
    )
endif()

# Install Python package files
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/kinex/"
    DESTINATION kinex
    FILES_MATCHING PATTERN "*.py"
)
