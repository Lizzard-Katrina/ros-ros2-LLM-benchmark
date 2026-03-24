#!/bin/bash
set -e

TARGET=../ros2_code/execute_task.cpp

echo "[TEST] Static check: task.plan()"

grep -q "task.plan" $TARGET || {
  echo "❌ task.plan() not found"
  exit 1
}

echo "✅ task.plan() found"

echo "[TEST] Static check: task.execute() (optional)"

if grep -q "task.execute" $TARGET; then
  echo "✅ task.execute() found"
else
  echo "⚠️ task.execute() not found (allowed but suboptimal)"
fi
