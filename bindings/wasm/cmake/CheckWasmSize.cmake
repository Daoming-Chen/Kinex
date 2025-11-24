if(NOT DEFINED KINEX_WASM_PATH)
    message(FATAL_ERROR "KINEX_WASM_PATH is not set. Cannot verify WebAssembly binary size.")
endif()

if(NOT EXISTS "${KINEX_WASM_PATH}")
    message(FATAL_ERROR "Expected WebAssembly binary not found at ${KINEX_WASM_PATH}.")
endif()

set(KINEX_WASM_MAX_SIZE 2097152)
file(SIZE "${KINEX_WASM_PATH}" KINEX_WASM_SIZE)

if(KINEX_WASM_SIZE GREATER KINEX_WASM_MAX_SIZE)
    math(EXPR KINEX_WASM_SIZE_KB "${KINEX_WASM_SIZE} / 1024")
    math(EXPR KINEX_WASM_MAX_SIZE_KB "${KINEX_WASM_MAX_SIZE} / 1024")
    message(FATAL_ERROR "kinex.wasm size ${KINEX_WASM_SIZE} bytes (${KINEX_WASM_SIZE_KB} KiB) exceeds limit of ${KINEX_WASM_MAX_SIZE} bytes (${KINEX_WASM_MAX_SIZE_KB} KiB).")
else()
    message(STATUS "kinex.wasm size ${KINEX_WASM_SIZE} bytes within limit ${KINEX_WASM_MAX_SIZE} bytes.")
endif()
