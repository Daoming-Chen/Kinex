cmake_minimum_required(VERSION 3.20)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "wasm bindings require the Emscripten toolchain. Configure the project with emcmake.")
endif()

set(KINEX_WASM_TARGET kinex_wasm)

add_executable(${KINEX_WASM_TARGET}
    src/bindings.cpp
)

set_target_properties(${KINEX_WASM_TARGET} PROPERTIES
    OUTPUT_NAME "kinex"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm
)

target_compile_definitions(${KINEX_WASM_TARGET}
    PRIVATE
        KINEX_WASM_BUILD
)

target_include_directories(${KINEX_WASM_TARGET}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${KINEX_WASM_TARGET}
    PRIVATE
        kinex
)

target_compile_options(${KINEX_WASM_TARGET}
    PRIVATE
        -O3
        -msimd128
)

target_link_options(${KINEX_WASM_TARGET}
    PRIVATE
        -O3
        --bind
        -sMODULARIZE=1
        -sEXPORT_NAME=createKinexModule
        -sALLOW_MEMORY_GROWTH=1
        -sENVIRONMENT=web,worker,node
        -sFILESYSTEM=0
        -sASSERTIONS=0
        -sERROR_ON_UNDEFINED_SYMBOLS=1
        -sEXPORTED_RUNTIME_METHODS=['UTF8ToString','lengthBytesUTF8']
        -sALLOW_TABLE_GROWTH
)

set(KINEX_WASM_OUTPUT_DIR ${CMAKE_BINARY_DIR}/wasm)
set(KINEX_WASM_BINARY ${KINEX_WASM_OUTPUT_DIR}/kinex.wasm)

add_custom_command(TARGET ${KINEX_WASM_TARGET} POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DKINEX_WASM_PATH=${KINEX_WASM_BINARY}
        -P ${PROJECT_SOURCE_DIR}/bindings/wasm/cmake/CheckWasmSize.cmake
    BYPRODUCTS ${KINEX_WASM_BINARY}
    COMMENT "Checking kinex.wasm binary size"
)

install(FILES
    ${KINEX_WASM_OUTPUT_DIR}/kinex.js
    ${KINEX_WASM_OUTPUT_DIR}/kinex.wasm
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/kinex/wasm
)

install(FILES
    ${PROJECT_SOURCE_DIR}/bindings/wasm/kinex.d.ts
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/kinex/wasm
)
