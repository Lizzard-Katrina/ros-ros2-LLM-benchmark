#!/usr/bin/env bash
set -e

echo "[setup] Sourcing ROS Noetic..."
source /opt/ros/noetic/setup.bash

echo "[setup] Setting PYTHONPATH..."
export PYTHONPATH=/root/ws_rosserial/src:$PYTHONPATH

echo "[setup] Ready."
exec "$@"
