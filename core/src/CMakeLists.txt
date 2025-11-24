# Core library sources
set(URDFX_SOURCES
    robot_model.cpp
    urdf_parser.cpp
    kinematics.cpp
    inverse_kinematics.cpp
)

set(URDFX_HEADERS
    ${PROJECT_SOURCE_DIR}/include/kinex/logging.h
    ${PROJECT_SOURCE_DIR}/include/kinex/robot_model.h
    ${PROJECT_SOURCE_DIR}/include/kinex/urdf_parser.h
    ${PROJECT_SOURCE_DIR}/include/kinex/kinematics.h
    ${PROJECT_SOURCE_DIR}/include/kinex/inverse_kinematics.h
)

# Create core library
add_library(kinex ${URDFX_SOURCES} ${URDFX_HEADERS})

# Set library properties
set_target_properties(kinex PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${URDFX_HEADERS}"
)

# Include directories
target_include_directories(kinex
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
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
        URDFX_BUILD
)

if(BUILD_SHARED_LIBS)
    target_compile_definitions(kinex
        PUBLIC
            URDFX_SHARED
    )
endif()
