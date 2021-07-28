#!/bin/bash

# Analyze code under DIRECTORY using Clang-Tidy
# Usage:
#   cd autoware.proj
#   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#   ./scripts/clang-tidy-package.sh DIRECTORY

USAGE="Usage:\n   ./scripts/clang-tidy-package.sh DIRECTORY"
EXAMPLE="Example:\n   ./scripts/clang-tidy-package.sh src/autoware/autoware.iv/common/util/autoware_utils/"

if [ $# != 1 ]; then
    echo "Error: missing argument"
    echo -e "${USAGE}"
    echo -e "${EXAMPLE}"
    exit 1
fi

if [ ! -d "${1}" ]; then
    echo "Error: ${1} not found"
    echo -e "${USAGE}"
    echo -e "${EXAMPLE}"
    exit 1
fi

if ! (type clang-tidy > /dev/null 2>&1); then
    echo "Error: missing Clang-Tidy"
    echo -e "Please install Clang-Tidy:\n   sudo apt install clang-tidy"
    exit 1
fi

if [ ! -d ./build ]; then
    echo "Error: missing build/ directory"
    echo "Please build first."
    exit 1
fi

if [ ! -f ./build/compile_commands.json ]; then
    echo "Error: missing build/compile_commands.json"
    echo "Please build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON option."
    echo -e "Example:\n   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    exit 1
fi

find "${1}" \
     -regex ".*\(cpp\|hpp\)" -print0 \
     -or -path "*test*" -prune \
     | xargs -0 clang-tidy -p build/
