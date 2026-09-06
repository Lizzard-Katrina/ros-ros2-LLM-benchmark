#!/usr/bin/env python3
"""Docker build + Tier3 runtime check against whatever is CURRENTLY sitting in
ros2_code/source/ and tests/test_runtime_ros2.py -- no LLM call, no retries,
no OPENROUTER_API_KEY needed. Use this after hand-editing a task's package or
its test_runtime_ros2.py, to quickly re-verify without touching the LLM pipeline.

Usage:
    python3 groundtruth_pipeline/verify_only.py --task interface_level/parameter_server/task_004_turtlrbot3_params
    python3 groundtruth_pipeline/verify_only.py --all               # every task that has ros2_code/source
    python3 groundtruth_pipeline/verify_only.py --file groundtruth_pipeline/needs_rerun.txt
"""

import argparse
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from docker_verify import verify_package

REPO_ROOT = Path(__file__).resolve().parent.parent


def find_all_tasks():
    tasks = []
    for root, dirs, _ in os.walk(REPO_ROOT):
        if "ros1_code" in dirs and not root.endswith("/source"):
            t = Path(root)
            if (t / "ros2_code" / "source").exists():
                tasks.append(t)
    return sorted(tasks)


def verify_one(task_dir: Path, update_status: bool):
    gt_source = task_dir / "ros2_code" / "source"
    harness_path = task_dir / "tests" / "test_runtime_ros2.py"

    if not gt_source.exists():
        return {"status": "NO_SOURCE", "detail": "ros2_code/source/ does not exist"}
    if not harness_path.exists():
        return {"status": "NO_HARNESS", "detail": "tests/test_runtime_ros2.py does not exist"}

    tmp_dir = tempfile.mkdtemp(prefix=f"verify_{task_dir.name}_")
    try:
        shutil.copytree(gt_source, tmp_dir, dirs_exist_ok=True)
        shutil.copy(harness_path, Path(tmp_dir) / "test_runtime_ros2.py")
        result = verify_package(tmp_dir, task_dir.name)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    passed = result.build_ok and result.test_ok
    detail = {
        "status": "GROUNDTRUTH_VERIFIED" if passed else "FAILED",
        "build_ok": result.build_ok,
        "test_ok": result.test_ok,
        "stage_failed": result.stage_failed,
        "build_log_tail": result.build_log[-3000:] if not result.build_ok else None,
        "test_log_tail": result.test_log[-3000:] if result.build_ok and not result.test_ok else None,
    }

    if update_status:
        status_path = task_dir / "ros2_code" / "GROUNDTRUTH_STATUS.json"
        payload = {
            "status": detail["status"],
            "verified_by": "manual (verify_only.py, no LLM)",
            "build_ok": result.build_ok,
            "test_ok": result.test_ok,
            "stage_failed": result.stage_failed,
        }
        status_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    return detail


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--file", type=str, help="text file, one task path per line")
    parser.add_argument("--no-update-status", action="store_true",
                         help="don't overwrite ros2_code/GROUNDTRUTH_STATUS.json, just print the result")
    args = parser.parse_args()

    if args.task:
        task_dirs = [Path(args.task)]
    elif args.file:
        task_dirs = [Path(l.strip()) for l in open(args.file) if l.strip()]
    elif args.all:
        task_dirs = find_all_tasks()
    else:
        parser.error("pass --task <dir>, --file <list.txt>, or --all")
        return

    passed, failed = 0, []
    for i, task_dir in enumerate(task_dirs):
        print(f"[{i+1}/{len(task_dirs)}] {task_dir} ...", end=" ", flush=True)
        detail = verify_one(task_dir, update_status=not args.no_update_status)
        print(detail["status"])
        if detail["status"] == "GROUNDTRUTH_VERIFIED":
            passed += 1
        else:
            failed.append(str(task_dir))
            tail = detail.get("build_log_tail") or detail.get("test_log_tail") or detail.get("detail") or ""
            if tail:
                print("  ---", (tail if isinstance(tail, str) else str(tail))[-800:])

    print(f"\n{passed}/{len(task_dirs)} passed")
    if failed:
        print("Still failing:")
        for f in failed:
            print("  ", f)


if __name__ == "__main__":
    main()
