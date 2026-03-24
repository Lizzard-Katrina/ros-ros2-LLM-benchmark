#!/bin/bash
set -e

echo "Starting Panda pick-and-place demo"

# 这里未来可以换成 roslaunch / rosrun
# 现在先假设你已经有一个 binary 或 demo

./build/panda_pick_place_demo
