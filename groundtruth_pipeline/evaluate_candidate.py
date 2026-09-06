#!/usr/bin/env python3
"""Evaluates a CANDIDATE model's translation against a task's verified groundtruth,
using the task's own Dockerfile_real -- the EXACT SAME reproducible artifact used to
verify groundtruth itself, just pointed at a build context where the target file(s)
have been swapped for the candidate's version. Nothing else about the build changes.

This is the actual benchmark scoring step: take the file(s) a model produced to
fill in ros1_code's TODOs (normally written to ros2_code/<file> by run_all.py /
run_all_5.py), drop them into the SAME package skeleton + dependencies + custom
msg/srv + mock support files that the groundtruth was verified against (everything
under ros2_code/source/ EXCEPT the target file(s) themselves), and run the SAME
tests/test_runtime_ros2.py harness via `docker build -f Dockerfile_real` + `docker run`.

Usage:
    # evaluate whatever is currently sitting in ros2_code/<file> for a task
    # (i.e. wherever run_all.py / run_all_5.py last wrote a model's translation)
    python3 groundtruth_pipeline/evaluate_candidate.py --task interface_level/service_client/task_003_mp3_db_service

    # evaluate a specific results directory from a run_all_5.py multi-run
    python3 groundtruth_pipeline/evaluate_candidate.py --task <dir> --candidate-dir interface_level/results3/some_model/service_client/task_003_mp3_db_service

    # batch: evaluate all verified tasks' current ros2_code/<file> in one go
    python3 groundtruth_pipeline/evaluate_candidate.py --all
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

from extract_candidate import extract_files_from_dialogue, materialize
from function_splice import find_hollowed_function_name, splice_function_into_groundtruth

REPO_ROOT = Path(__file__).resolve().parent.parent
CODE_EXT = (".py", ".cpp", ".hpp", ".h", ".cc")


def target_filenames(task_dir: Path) -> set:
    """Files a candidate model is actually responsible for -- same basenames as
    the hollowed ros1_code/*.py|cpp files that were shown to the model."""
    ros1_dir = task_dir / "ros1_code"
    return {
        f.replace("_todo", "") for f in os.listdir(ros1_dir)
        if f.endswith(CODE_EXT) and (ros1_dir / f).is_file()
    }

def is_groundtruth_verified(task_dir: Path) -> bool:
    status_path = task_dir / "ros2_code" / "GROUNDTRUTH_STATUS.json"
    if not status_path.exists():
        return False
    try:
        return json.loads(status_path.read_text()).get("status") == "GROUNDTRUTH_VERIFIED"
    except Exception:
        return False


def build_hybrid_context(task_dir: Path, candidate_dir: Path):
    """Mirrors the task's real directory layout (ros2_code/source/, tests/,
    Dockerfile_real) in a temp dir, with the candidate's file(s) swapped in for
    the groundtruth's own copy of each target file -- everything else identical."""
    tmp_dir = Path(tempfile.mkdtemp(prefix=f"eval_{task_dir.name}_"))
    dest_source = tmp_dir / "ros2_code" / "source"
    shutil.copytree(task_dir / "ros2_code" / "source", dest_source)
    shutil.copytree(task_dir / "tests", tmp_dir / "tests")
    shutil.copy(task_dir / "Dockerfile_real", tmp_dir / "Dockerfile_real")

    targets = target_filenames(task_dir)
    missing = []
    for fname in targets:
        candidate_path = candidate_dir / fname
        if not candidate_path.exists():
            missing.append(fname)
            continue
        content = candidate_path.read_text(encoding="utf-8", errors="ignore")
        found_any = False
        for p in dest_source.rglob(fname):
            p.write_text(content, encoding="utf-8")
            found_any = True
        if not found_any:
            (dest_source / fname).write_text(content, encoding="utf-8")
    return tmp_dir, missing


def build_function_level_context(task_dir: Path, candidate_dir: Path):
    """Diagnostic variant of build_hybrid_context: instead of swapping the whole
    target file, spice ONLY the function/method that contained the TODO into an
    otherwise-untouched groundtruth file. Only works when that function's name
    survived translation unchanged on both the groundtruth and candidate side --
    common when the hollowed code was already a class method (e.g. C++ node
    classes), but frequently NOT possible when a ROS1 standalone function got
    refactored into a ROS2 Node class method (a very common, legitimate pattern).
    Returns (tmp_dir, missing, not_applicable) -- not_applicable lists target
    files where splicing could not be done, so the caller can report NA honestly
    instead of silently mis-scoring them."""
    tmp_dir = Path(tempfile.mkdtemp(prefix=f"evalfn_{task_dir.name}_"))
    dest_source = tmp_dir / "ros2_code" / "source"
    shutil.copytree(task_dir / "ros2_code" / "source", dest_source)
    shutil.copytree(task_dir / "tests", tmp_dir / "tests")
    shutil.copy(task_dir / "Dockerfile_real", tmp_dir / "Dockerfile_real")

    targets = target_filenames(task_dir)
    missing, not_applicable = [], []
    for fname in targets:
        candidate_path = candidate_dir / fname
        if not candidate_path.exists():
            missing.append(fname)
            continue
        ros1_content = (task_dir / "ros1_code" / fname).read_text(encoding="utf-8", errors="ignore")
        func_name = find_hollowed_function_name(ros1_content, fname)
        if func_name is None:
            not_applicable.append(fname)
            continue
        candidate_content = candidate_path.read_text(encoding="utf-8", errors="ignore")
        spliced_any = False
        for p in dest_source.rglob(fname):
            gt_content = p.read_text(encoding="utf-8", errors="ignore")
            spliced = splice_function_into_groundtruth(gt_content, candidate_content, func_name, fname)
            if spliced is None:
                continue
            p.write_text(spliced, encoding="utf-8")
            spliced_any = True
        if not spliced_any:
            not_applicable.append(fname)
    return tmp_dir, missing, not_applicable


def _docker_build_and_run(ctx_dir: Path, image_tag: str):
    build = subprocess.run(
        ["docker", "build", "-t", image_tag, "-f", str(ctx_dir / "Dockerfile_real"), str(ctx_dir)],
        capture_output=True, encoding="utf-8", errors="replace", timeout=900,
    )
    if build.returncode != 0:
        return "FAIL", "build", (build.stdout + build.stderr)[-2500:]
    run = subprocess.run(
        ["docker", "run", "--rm", image_tag],
        capture_output=True, encoding="utf-8", errors="replace", timeout=200,
    )
    if run.returncode != 0:
        return "FAIL", "test", (run.stdout + run.stderr)[-2500:]
    return "PASS", None, None


def evaluate_task_function_level(task_dir: Path, candidate_dir: Path = None, dialogue_md: Path = None, verbose=True):
    """Diagnostic metric (see build_function_level_context). Returns NOT_APPLICABLE
    when none of the target files could be function-spliced."""
    if not is_groundtruth_verified(task_dir):
        return {"status": "NO_GROUNDTRUTH"}
    if not (task_dir / "Dockerfile_real").exists():
        return {"status": "NO_DOCKERFILE"}

    extracted_tmp = None
    if dialogue_md is not None:
        targets_for_fallback = target_filenames(task_dir)
        fallback = next(iter(targets_for_fallback)) if len(targets_for_fallback) == 1 else None
        files = extract_files_from_dialogue(dialogue_md, fallback_filename=fallback)
        if not files:
            return {"status": "NO_CANDIDATE_FILES"}
        extracted_tmp = Path(tempfile.mkdtemp(prefix="dialogue_extract_"))
        materialize(files, extracted_tmp)
        candidate_dir = extracted_tmp
    candidate_dir = candidate_dir or (task_dir / "ros2_code")

    try:
        ctx_dir, missing, not_applicable = build_function_level_context(task_dir, candidate_dir)
        targets = target_filenames(task_dir)
        if len(not_applicable) + len(missing) == len(targets):
            shutil.rmtree(ctx_dir, ignore_errors=True)
            return {"status": "NOT_APPLICABLE", "not_applicable_files": not_applicable, "missing_target_files": missing}

        image_tag = f"evalfn-{task_dir.name}-{uuid.uuid4().hex[:8]}"
        try:
            status, stage_failed, log_tail = _docker_build_and_run(ctx_dir, image_tag)
            if verbose:
                print(f"  [function-level] {status}")
            return {
                "status": status, "stage_failed": stage_failed,
                "not_applicable_files": not_applicable, "missing_target_files": missing,
                "log_tail": log_tail,
            }
        finally:
            subprocess.run(["docker", "rmi", "-f", image_tag], capture_output=True)
            shutil.rmtree(ctx_dir, ignore_errors=True)
    finally:
        if extracted_tmp:
            shutil.rmtree(extracted_tmp, ignore_errors=True)


def evaluate_task(task_dir: Path, candidate_dir: Path = None, dialogue_md: Path = None, verbose=True):
    if not is_groundtruth_verified(task_dir):
        return {"status": "NO_GROUNDTRUTH", "detail": "task has no GROUNDTRUTH_VERIFIED baseline to evaluate against"}
    if not (task_dir / "Dockerfile_real").exists():
        return {"status": "NO_DOCKERFILE", "detail": "task has no Dockerfile_real -- run generate_dockerfiles.py first"}

    extracted_tmp = None
    if dialogue_md is not None:
        targets_for_fallback = target_filenames(task_dir)
        fallback = next(iter(targets_for_fallback)) if len(targets_for_fallback) == 1 else None
        files = extract_files_from_dialogue(dialogue_md, fallback_filename=fallback)
        if not files:
            return {"status": "NO_CANDIDATE_FILES", "detail": f"no [FILENAME: ...] blocks parsed from {dialogue_md}"}
        extracted_tmp = Path(tempfile.mkdtemp(prefix="dialogue_extract_"))
        materialize(files, extracted_tmp)
        candidate_dir = extracted_tmp

    candidate_dir = candidate_dir or (task_dir / "ros2_code")
    targets = target_filenames(task_dir)
    if not any((candidate_dir / f).exists() for f in targets):
        if extracted_tmp:
            shutil.rmtree(extracted_tmp, ignore_errors=True)
        return {"status": "NO_CANDIDATE_FILES", "detail": f"none of {sorted(targets)} found in {candidate_dir}"}

    ctx_dir, missing = build_hybrid_context(task_dir, candidate_dir)
    image_tag = f"eval-{task_dir.name}-{uuid.uuid4().hex[:8]}"
    try:
        status, stage_failed, log_tail = _docker_build_and_run(ctx_dir, image_tag)
        if verbose:
            print(f"  {status}" + (f" (missing target files: {missing})" if missing else ""))
        return {
            "status": status, "stage_failed": stage_failed,
            "missing_target_files": missing, "log_tail": log_tail,
        }
    finally:
        subprocess.run(["docker", "rmi", "-f", image_tag], capture_output=True)
        shutil.rmtree(ctx_dir, ignore_errors=True)
        if extracted_tmp:
            shutil.rmtree(extracted_tmp, ignore_errors=True)


def find_all_tasks():
    tasks = []
    for root, dirs, _ in os.walk(REPO_ROOT):
        if "ros1_code" in dirs and not root.endswith("/source"):
            tasks.append(Path(root))
    return sorted(tasks)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, help="single task directory")
    parser.add_argument("--candidate-dir", type=str, default=None,
                         help="directory containing the candidate's translated file(s); "
                              "defaults to <task>/ros2_code/")
    parser.add_argument("--candidate-dialogue", type=str, default=None,
                         help="path to a run_all_5.py-style dialogue.md to extract the candidate's "
                              "file(s) from directly, instead of --candidate-dir")
    parser.add_argument("--all", action="store_true", help="evaluate every GROUNDTRUTH_VERIFIED task's current ros2_code/")
    parser.add_argument("--with-function-diagnostic", action="store_true",
                         help="also run the function-splice-only diagnostic metric alongside the "
                              "primary whole-file metric (best-effort; NOT_APPLICABLE where the "
                              "hollowed function's name/shape didn't survive translation unchanged)")
    parser.add_argument("--out", type=str, default="candidate_eval_results.json")
    args = parser.parse_args()

    if args.task:
        task_dirs = [Path(args.task)]
    elif args.all:
        task_dirs = [t for t in find_all_tasks() if is_groundtruth_verified(t)]
    else:
        parser.error("pass --task <dir> or --all")
        return

    results = {}
    for i, task_dir in enumerate(task_dirs):
        print(f"[{i+1}/{len(task_dirs)}] {task_dir}")
        cand_dir = Path(args.candidate_dir) if args.candidate_dir else None
        dialogue = Path(args.candidate_dialogue) if args.candidate_dialogue else None
        entry = evaluate_task(task_dir, cand_dir, dialogue)
        if args.with_function_diagnostic:
            entry["function_level_diagnostic"] = evaluate_task_function_level(task_dir, cand_dir, dialogue)
        results[str(task_dir)] = entry

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    passed = sum(1 for r in results.values() if r["status"] == "PASS")
    print(f"\n{passed}/{len(results)} candidate submissions passed (build+runtime match against groundtruth)")
    print(f"Full results written to {args.out}")


if __name__ == "__main__":
    main()
