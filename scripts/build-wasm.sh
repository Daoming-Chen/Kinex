#!/bin/bash
# Build WASM for kinex
# This script sets up Emscripten environment and builds WASM

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

function write_step {
    echo -e "\n${CYAN}===> $1${NC}"
}

function write_success {
    echo -e "${GREEN}✓ $1${NC}"
}

function write_error {
    echo -e "${RED}✗ $1${NC}"
}

# Get script directory and project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

echo -e "${CYAN}╔════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  kinex WASM Builder for Linux/Mac      ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════╝${NC}"

# Check for Emscripten SDK
write_step "Setting up Emscripten environment..."
EMSDK_DIR="$PROJECT_ROOT/third_party/emsdk"

if [ ! -d "$EMSDK_DIR" ]; then
    write_error "Emscripten SDK not found at $EMSDK_DIR"
    exit 1
fi

# Source emsdk environment
if [ -f "$EMSDK_DIR/emsdk_env.sh" ]; then
    source "$EMSDK_DIR/emsdk_env.sh"
    write_success "Emscripten environment loaded"
else
    write_error "emsdk_env.sh not found"
    exit 1
fi

# Clean previous build
if [ -d "$PROJECT_ROOT/build-wasm" ]; then
    write_step "Cleaning previous build-wasm directory..."
    rm -rf "$PROJECT_ROOT/build-wasm"
    write_success "Cleaned previous build"
fi

# Configure with CMake
write_step "Configuring CMake..."
cd "$PROJECT_ROOT"
emcmake cmake -B build-wasm -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=Release
write_success "CMake configuration completed"

# Build
write_step "Building WASM..."
cmake --build build-wasm --config Release
write_success "WASM build completed"

# Verify output files
write_step "Verifying output files..."
WASM_OUTPUT_DIR="$PROJECT_ROOT/build-wasm/wasm"
if [ ! -f "$WASM_OUTPUT_DIR/kinex.js" ]; then
    write_error "kinex.js not found in $WASM_OUTPUT_DIR"
    exit 1
fi
if [ ! -f "$WASM_OUTPUT_DIR/kinex.wasm" ]; then
    write_error "kinex.wasm not found in $WASM_OUTPUT_DIR"
    exit 1
fi
write_success "Output files verified"

echo -e "\n${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  WASM build completed successfully! 🎉 ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo -e "\n${CYAN}Output files are in: $WASM_OUTPUT_DIR${NC}"
