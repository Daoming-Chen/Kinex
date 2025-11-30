# Core C++ library for kinex
cmake_minimum_required(VERSION 3.20)

# Core library sources
set(KINEX_SOURCES
    src/robot_model.cpp
    src/robot.cpp
    src/urdf_parser.cpp
    src/kinematics.cpp
    src/inverse_kinematics.cpp
    src/global_inverse_kinematics.cpp
)

set(KINEX_HEADERS
    include/kinex/logging.h
    include/kinex/robot_model.h
    include/kinex/robot.h
    include/kinex/urdf_parser.h
    include/kinex/kinematics.h
    include/kinex/inverse_kinematics.h
    include/kinex/global_inverse_kinematics.h
    include/kinex/export.h
)

# Create core library
add_library(kinex ${KINEX_SOURCES} ${KINEX_HEADERS})

# Set library properties
set_target_properties(kinex PROPERTIES
    PUBLIC_HEADER "${KINEX_HEADERS}"
)

# Only set version properties if not building Python bindings
# This avoids issues with versioned shared libraries in Python wheels
if(NOT BUILD_PYTHON_BINDINGS)
    set_target_properties(kinex PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )
endif()

if(APPLE)
    set_target_properties(kinex PROPERTIES
        INSTALL_RPATH "@loader_path"
        BUILD_RPATH "@loader_path"
    )
else()
    set_target_properties(kinex PROPERTIES
        INSTALL_RPATH "$ORIGIN"
        BUILD_RPATH "$ORIGIN"
    )
endif()

# Include directories
target_include_directories(kinex
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link dependencies
target_link_libraries(kinex
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog
    PRIVATE
        pugixml::pugixml
        daqp::daqp
)

# Compile definitions
target_compile_definitions(kinex
    PRIVATE
        KINEX_BUILD
)

if(BUILD_SHARED_LIBS)
    target_compile_definitions(kinex
        PUBLIC
            KINEX_SHARED
    )
endif()

# Installation rules
install(TARGETS kinex
    EXPORT kinexTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kinex
)

# Only export targets if all dependencies are system packages (not submodules)
# This avoids export errors when building with submodule dependencies
if(Eigen3_FOUND AND spdlog_FOUND AND pugixml_FOUND)
    # Export targets
    install(EXPORT kinexTargets
        FILE kinexTargets.cmake
        NAMESPACE kinex::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinex
    )
else()
    message(STATUS "Skipping CMake export (using submodule dependencies)")
endif()

# Tests
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
