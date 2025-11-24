cmake_minimum_required(VERSION 3.20)

# Define the Python extension module
# nanobind_add_module creates a shared library with the right suffix for Python
nanobind_add_module(_kinex src/bindings.cpp)

# Link against the core library only
# nanobind is linked automatically by nanobind_add_module
target_link_libraries(_kinex PRIVATE kinex)

# Set C++ standard (should inherit from parent but good to be explicit)
target_compile_features(_kinex PRIVATE cxx_std_20)

# Define version
target_compile_definitions(_kinex PRIVATE KINEX_VERSION="${PROJECT_VERSION}")

# Set output directory to bindings/python/kinex package folder
# This allows editable installs and testing to work easily
set_target_properties(_kinex PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/kinex
    OUTPUT_NAME _kinex
)

# Avoid "lib" prefix on Windows/Linux (Python expects _kinex.so/pyd not lib_kinex.so)
set_target_properties(_kinex PROPERTIES PREFIX "")

# Set RPATH to find libkinex.so in the same directory
# $ORIGIN means the directory containing the binary
if(APPLE)
    set_target_properties(_kinex PROPERTIES
        INSTALL_RPATH "@loader_path"
        BUILD_RPATH "@loader_path"
        BUILD_WITH_INSTALL_RPATH FALSE
    )
else()
    set_target_properties(_kinex PROPERTIES
        INSTALL_RPATH "$ORIGIN"
        BUILD_RPATH "$ORIGIN"
        BUILD_WITH_INSTALL_RPATH FALSE
    )
endif()

# For editable/development installs, copy all needed .so/.dll files to the Python package directory
# so the extension module can find them without needing to set LD_LIBRARY_PATH
if(WIN32)
    # Windows uses DLL files, no SONAME concept
    add_custom_command(TARGET _kinex POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:kinex>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_FILE_NAME:kinex>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:pugixml-shared>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_FILE_NAME:pugixml-shared>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:spdlog>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_FILE_NAME:spdlog>
        COMMENT "Copying shared libraries to Python package directory for editable install"
    )
else()
    # Unix-like systems use SONAME
    add_custom_command(TARGET _kinex POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:kinex>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_FILE_NAME:kinex>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_SONAME_FILE:kinex>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_SONAME_FILE_NAME:kinex>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:pugixml-shared>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_FILE_NAME:pugixml-shared>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_SONAME_FILE:pugixml-shared>
            ${CMAKE_CURRENT_SOURCE_DIR}/kinex/$<TARGET_SONAME_FILE_NAME:pugixml-shared>
        COMMENT "Copying shared libraries to Python package directory for editable install"
    )
endif()

# Installation rules
install(TARGETS _kinex
    LIBRARY DESTINATION kinex
    ARCHIVE DESTINATION kinex
    RUNTIME DESTINATION kinex
    COMPONENT python
)

# Also install libkinex.so and pugixml alongside the Python module
# spdlog is typically header-only or statically linked, so we don't install it
install(TARGETS kinex
    LIBRARY DESTINATION kinex
    ARCHIVE DESTINATION kinex
    RUNTIME DESTINATION kinex
    COMPONENT python
)

# Install pugixml if it's a shared library target
if(TARGET pugixml-shared)
    install(TARGETS pugixml-shared
        LIBRARY DESTINATION kinex
        ARCHIVE DESTINATION kinex
        RUNTIME DESTINATION kinex
        COMPONENT python
    )
endif()

# Install spdlog if it's a shared library target
if(TARGET spdlog AND BUILD_SHARED_LIBS)
    install(TARGETS spdlog
        LIBRARY DESTINATION kinex
        ARCHIVE DESTINATION kinex
        RUNTIME DESTINATION kinex
        COMPONENT python
    )
endif()

