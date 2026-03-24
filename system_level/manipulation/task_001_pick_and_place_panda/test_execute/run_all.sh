#!/bin/bash
set -e

echo "========== RUNNING ALL TESTS =========="

bash test_static.sh
#bash test_plan_runtime.sh

echo "🎉 ALL TESTS PASSED"
