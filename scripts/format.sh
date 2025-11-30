#!/usr/bin/env bash
set -euo pipefail

# Format all C/C++ files in the repo, excluding build and third_party
ROOT_DIR=$(git rev-parse --show-toplevel 2>/dev/null || echo "$(pwd)")
CLANG_FORMAT="clang-format-14"
if ! command -v "$CLANG_FORMAT" >/dev/null 2>&1; then
  if command -v clang-format >/dev/null 2>&1; then
    echo "Warning: clang-format-14 not found; falling back to 'clang-format' (version may vary)." >&2
    CLANG_FORMAT=clang-format
  else
    echo "Error: clang-format not found. Please install clang-format or clang-format-14." >&2
    exit 1
  fi
fi
echo "Using $CLANG_FORMAT to format C/C++ files under $ROOT_DIR"

find "$ROOT_DIR" -type f \( -iname '*.cpp' -o -iname '*.cc' -o -iname '*.c' -o -iname '*.h' -o -iname '*.hpp' -o -iname '*.hh' -o -iname '*.cxx' -o -iname '*.hxx' \) -not -path "$ROOT_DIR/build/*" -not -path "$ROOT_DIR/third_party/*" -not -path "$ROOT_DIR/.git/*" -print0 | xargs -0 "$CLANG_FORMAT" -i -style=file

echo "Format complete."
