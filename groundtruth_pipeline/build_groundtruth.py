#!/usr/bin/env python3
"""Builds and Docker-verifies (Tier1 build + Tier3 runtime) ROS2 groundtruth
translations for benchmark tasks, using an LLM via OpenRouter to do the
actual translation + fix-on-failure loop.

Usage:
    export OPENROUTER_API_KEY=...
    python3 groundtruth_pipeline/build_groundtruth.py --task interface_level/service_client/task_003_mp3_db_service
    python3 groundtruth_pipeline/build_groundtruth.py --all --limit 5
    python3 groundtruth_pipeline/build_groundtruth.py --all --skip-existing

What "success" means (Tier3, not Tier1): a task is only marked GROUNDTRUTH_VERIFIED
if colcon build succeeds AND test_runtime_ros2.py passes for real inside the
Docker container (an actual node was launched and produced correct output),
not merely "it compiled".
"""

import argparse
import json
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

# Some environments (WSL/conda with LANG unset) default to ASCII instead of
# UTF-8 for stdout/stderr, which crashes as soon as any LLM output or task
# file contains a non-ASCII character (curly quotes, em dashes, etc.).
for _stream in (sys.stdout, sys.stderr):
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="replace")

sys.path.insert(0, os.path.dirname(__file__))

from docker_verify import cleanup_package, materialize_package, verify_package
from openrouter_client import DEFAULT_MODEL, chat, get_client
from prompts import (SYSTEM_PROMPT, build_fix_prompt, build_initial_prompt,
                      parse_file_blocks)
from task_context import load_task_context

REPO_ROOT = Path(__file__).resolve().parent.parent

# Tasks flagged earlier as needing manual link review, or with no source at all,
# are not eligible for this automated pipeline.
NOT_ELIGIBLE_STATUSES = {"NEEDS_MANUAL_REVIEW"}


def is_eligible(task_dir: Path) -> bool:
    source_dir = task_dir / "ros1_code" / "source"
    info_path = source_dir / "SOURCE_INFO.json"
    if not source_dir.exists():
        # no cloned ROS1 reference at all -- still eligible, just translate
        # directly from the hollowed ros1_code with no extra context.
        return True
    if info_path.exists():
        try:
            info = json.loads(info_path.read_text())
            if info.get("status") in NOT_ELIGIBLE_STATUSES:
                return False
        except Exception:
            pass
    return True


def already_verified(task_dir: Path) -> bool:
    status_path = task_dir / "ros2_code" / "GROUNDTRUTH_STATUS.json"
    if not status_path.exists():
        return False
    try:
        return json.loads(status_path.read_text()).get("status") == "GROUNDTRUTH_VERIFIED"
    except Exception:
        return False


def find_all_tasks():
    tasks = []
    for root, dirs, files in os.walk(REPO_ROOT):
        if "ros1_code" in dirs and not root.endswith("/source"):
            tasks.append(Path(root))
    return sorted(tasks)


def persist_files(task_dir: Path, files: dict):
    dest_root = task_dir / "ros2_code" / "source"
    dest_root.mkdir(parents=True, exist_ok=True)
    harness_content = None
    for rel_path, content in files.items():
        # match by basename, not exact path -- the LLM may have used test/ or tests/
        # as a prefix even though instructed not to.
        if os.path.basename(rel_path) == "test_runtime_ros2.py":
            harness_content = content
            continue
        full_path = dest_root / rel_path
        full_path.parent.mkdir(parents=True, exist_ok=True)
        full_path.write_text(content, encoding="utf-8")

    if harness_content is not None:
        tests_dir = task_dir / "tests"
        tests_dir.mkdir(parents=True, exist_ok=True)
        (tests_dir / "test_runtime_ros2.py").write_text(harness_content, encoding="utf-8")


def write_status(task_dir: Path, status: str, detail: dict):
    # NOTE: don't blindly slice the serialized JSON string -- that can cut
    # the file off mid-structure and produce invalid JSON. Fields that can be
    # large (log tails, raw LLM output) are already bounded by the caller.
    out = {"status": status, **detail}
    (task_dir / "ros2_code").mkdir(parents=True, exist_ok=True)
    (task_dir / "ros2_code" / "GROUNDTRUTH_STATUS.json").write_text(
        json.dumps(out, indent=2), encoding="utf-8"
    )


def process_task(task_dir: Path, model: str, max_fix_rounds: int, verbose=True):
    if verbose:
        print(f"\n=== {task_dir} ===")

    ctx = load_task_context(str(task_dir))
    client = get_client()
    package_name = task_dir.name

    prompt = build_initial_prompt(ctx)
    files = {}
    attempts = []

    for round_idx in range(max_fix_rounds + 1):
        if round_idx == 0:
            raw = chat(client, SYSTEM_PROMPT, prompt, model=model)
        else:
            raw = chat(client, SYSTEM_PROMPT, fix_prompt, model=model)
        new_files = parse_file_blocks(raw)
        if not new_files:
            write_status(task_dir, "LLM_NO_FILES_PARSED", {"round": round_idx, "raw_excerpt": raw[:2000]})
            return "LLM_NO_FILES_PARSED"
        files = new_files

        pkg_dir = materialize_package(files, package_name)
        try:
            result = verify_package(pkg_dir, package_name)
        finally:
            cleanup_package(pkg_dir)

        attempts.append({
            "round": round_idx,
            "build_ok": result.build_ok,
            "test_ok": result.test_ok,
            "stage_failed": result.stage_failed,
            "build_log_tail": result.build_log[-2500:] if not result.build_ok else None,
            "test_log_tail": result.test_log[-2500:] if result.build_ok and not result.test_ok else None,
        })

        if result.build_ok and result.test_ok:
            persist_files(task_dir, files)
            write_status(task_dir, "GROUNDTRUTH_VERIFIED", {
                "model": model,
                "rounds_needed": round_idx + 1,
                "attempts": attempts,
            })
            if verbose:
                print(f"  PASSED after {round_idx + 1} round(s)")
            return "GROUNDTRUTH_VERIFIED"

        stage = result.stage_failed
        log = result.build_log if stage == "build" else result.test_log
        if verbose:
            print(f"  round {round_idx}: FAILED at {stage}")
        fix_prompt = build_fix_prompt(files, stage, log)

    # exhausted retries: persist the last attempt anyway for human review, but mark unverified
    persist_files(task_dir, files)
    write_status(task_dir, "GROUNDTRUTH_UNVERIFIED_MAX_RETRIES", {
        "model": model,
        "attempts": attempts,
        "note": "Best-effort files were written to ros2_code/source and tests/test_runtime_ros2.py "
                "for human review, but did not pass Tier3 within the retry budget.",
    })
    return "GROUNDTRUTH_UNVERIFIED_MAX_RETRIES"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, help="path to a single task directory")
    parser.add_argument("--all", action="store_true", help="process all eligible tasks")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--skip-existing", action="store_true", help="skip tasks already GROUNDTRUTH_VERIFIED")
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL)
    parser.add_argument("--max-fix-rounds", type=int, default=4)
    args = parser.parse_args()

    if args.task:
        task_dirs = [Path(args.task)]
    elif args.all:
        task_dirs = [t for t in find_all_tasks() if is_eligible(t)]
        if args.skip_existing:
            task_dirs = [t for t in task_dirs if not already_verified(t)]
        if args.limit:
            task_dirs = task_dirs[: args.limit]
    else:
        parser.error("pass --task <dir> or --all")
        return

    print(f"Processing {len(task_dirs)} task(s) with model={args.model}")
    summary = {}
    for i, task_dir in enumerate(task_dirs):
        print(f"[{i+1}/{len(task_dirs)}]", end=" ")
        try:
            status = process_task(task_dir, args.model, args.max_fix_rounds)
        except Exception as e:
            tb = traceback.format_exc()
            status = f"PIPELINE_ERROR: {e}"
            print(tb)
            write_status(task_dir, "PIPELINE_ERROR", {"error": str(e), "traceback": tb})
        summary[str(task_dir)] = status

    ok = sum(1 for v in summary.values() if v == "GROUNDTRUTH_VERIFIED")
    print(f"\n{ok}/{len(summary)} tasks reached GROUNDTRUTH_VERIFIED (Tier3 passed)")
    for k, v in summary.items():
        if v != "GROUNDTRUTH_VERIFIED":
            print(f"  NOT VERIFIED: {k} -> {v}")


if __name__ == "__main__":
    main()
