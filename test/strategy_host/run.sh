#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
strategy_test_dir=$(mktemp -d)
trap 'rm -rf "$strategy_test_dir"' EXIT HUP INT TERM
c++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined \
    -Itest/strategy_host/stubs -Isrc \
    test/strategy_host/strategy_test.cpp src/strategy/strategy.cpp \
    src/util/PID.cpp src/util/Vector.cpp -o "$strategy_test_dir/strategy_test"
"$strategy_test_dir/strategy_test"
c++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined \
    -Itest/strategy_host/stubs -Isrc \
    test/strategy_host/preset_test.cpp src/util/StartingPreset.cpp \
    -o "$strategy_test_dir/preset_test"
"$strategy_test_dir/preset_test"
c++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined \
    -Itest/strategy_host/stubs -Isrc \
    test/strategy_host/odometry_test.cpp src/odometry/Odometry.cpp src/util/Vector.cpp \
    -o "$strategy_test_dir/odometry_test"
"$strategy_test_dir/odometry_test"
c++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined \
    -Itest/strategy_host/stubs -Isrc \
    test/strategy_host/boundary_tracking_test.cpp src/strategy/strategy.cpp \
    src/odometry/Odometry.cpp src/colour/colour.cpp src/colour/ColourModule/ColourModule.cpp \
    src/util/PID.cpp src/util/Vector.cpp -o "$strategy_test_dir/boundary_tracking_test"
"$strategy_test_dir/boundary_tracking_test"
