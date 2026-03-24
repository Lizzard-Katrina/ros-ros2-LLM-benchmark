#!/bin/bash
set -e

LOG=/tmp/panda_plan.log

echo "[TEST] Runtime planning test"

# 假设你有一个 demo 可执行程序
# 例如: rosrun panda_pick_place demo
timeout 30s ./run_demo.sh > $LOG 2>&1 || true

echo "[TEST] Checking for crash"
grep -E "Segmentation fault|Exception|FATAL" $LOG && {
  echo "❌ Runtime crash detected"
  exit 1
}

echo "[TEST] Checking for planning success signal"
grep -E "plan|Planning|solution|succeeded" $LOG || {
  echo "❌ No planning success signal found"
  exit 1
}

echo "✅ Runtime planning test passed"
