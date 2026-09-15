#!/usr/bin/env bash
# No-CMake fallback. Usage: ./build.sh  [then ./adas_sim --seconds=60 --seed=42]
set -e
CXX=${CXX:-g++}
$CXX -std=c++20 -O2 -Wall -Wextra -Wpedantic -Iinclude -pthread src/main.cpp    -o adas_sim
$CXX -std=c++20 -O2 -Wall -Wextra -Wpedantic -Iinclude -pthread tests/test_main.cpp -o adas_tests
echo "built: ./adas_sim ./adas_tests"
