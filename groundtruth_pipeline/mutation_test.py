#!/usr/bin/env python3
"""Negative-control ("mutation") test: for each GROUNDTRUTH_VERIFIED task, gut the
logic that was originally hollowed (the TODO region) -- replace it with a harmless,
syntactically-valid no-op stub that still compiles but does nothing meaningful --
then re-run the SAME Dockerfile_real.

If the test STILL PASSES after the real logic has been removed, that is definitive,
mechanical proof the test never actually depended on that logic in the first place
(a "shadow" test) -- no human code reading required to reach this conclusion.
If the test FAILS (as it should), that's positive evidence the test is real.

This only works where the hollowed function's name survived translation unchanged
(same limitation as function_splice.py's other diagnostic use) -- tasks where it
doesn't are honestly marked MUTATION_NOT_APPLICABLE, not silently skipped as if
verified.

Usage:
    python3 groundtruth_pipeline/mutation_test.py --all
    python3 groundtruth_pipeline/mutation_test.py --task interface_level/service_client/task_003_mp3_db_service
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
from function_splice import find_hollowed_function_name, _py_extract_function, _cpp_extract_function, is_cpp

REPO_ROOT = Path(__file__).resolve().parent.parent
CODE_EXT = (".py", ".cpp", ".hpp", ".h", ".cc")

PY_STUB_BODY_TEMPLATE = "{indent}def {name}({params}):\n{indent}    pass\n"
CPP_STUB_SIGNATURE_KEEP = True  # keep the original signature line, just gut the body


def target_filenames(task_dir: Path) -> set:
    ros1_dir = task_dir / "ros1_code"
    return {f for f in os.listdir(ros1_dir) if f.endswith(CODE_EXT) and (ros1_dir / f).is_file()}


def is_groundtruth_verified(task_dir: Path) -> bool:
    status_path = task_dir / "ros2_code" / "GROUNDTRUTH_STATUS.json"
    if not status_path.exists():
        return False
    try:
        return json.loads(status_path.read_text()).get("status") == "GROUNDTRUTH_VERIFIED"
    except Exception:
        return False


def gut_function(content: str, func_name: str, fname: str):
    """Returns mutated content with func_name's body replaced by a no-op stub,
    or None if the function couldn't be located (name didn't survive translation)."""
    lines = content.splitlines()
    if is_cpp(fname):
        loc = _cpp_extract_function(content, func_name)
        if loc is None:
            return None
        start, end, block = loc
        sig_line = lines[start]
        # keep the signature line (with its opening brace), drop everything else,
        # close with a bare return so it compiles regardless of declared return type
        # for void functions; for non-void this may fail to build, which is fine --
        # a build failure is still a valid (if less clean) signal, not a false PASS.
        stub = [sig_line if "{" in sig_line else sig_line + " {", "  return;", "}"]
        new_lines = lines[:start] + stub + lines[end + 1:]
        return "\n".join(new_lines)
    else:
        loc = _py_extract_function(content, func_name)
        if loc is None:
            return None
        start, end, block = loc
        def_line = lines[start]
        indent = def_line[: len(def_line) - len(def_line.lstrip())]
        new_lines = lines[:start] + [def_line, indent + "    pass"] + lines[end + 1:]
        return "\n".join(new_lines)


def build_mutated_context(task_dir: Path):
    """Returns (ctx_dir, mutated_files, not_applicable) -- mutated_files is the list
    of target filenames that were successfully gutted."""
    tmp_dir = Path(tempfile.mkdtemp(prefix=f"mutate_{task_dir.name}_"))
    dest_source = tmp_dir / "ros2_code" / "source"
    shutil.copytree(task_dir / "ros2_code" / "source", dest_source)
    shutil.copytree(task_dir / "tests", tmp_dir / "tests")
    shutil.copy(task_dir / "Dockerfile_real", tmp_dir / "Dockerfile_real")

    mutated, not_applicable = [], []
    for fname in target_filenames(task_dir):
        ros1_content = (task_dir / "ros1_code" / fname).read_text(encoding="utf-8", errors="ignore")
        func_name = find_hollowed_function_name(ros1_content, fname)
        if func_name is None:
            not_applicable.append(fname)
            continue
        did_mutate = False
        for p in dest_source.rglob(fname):
            gt_content = p.read_text(encoding="utf-8", errors="ignore")
            mutated_content = gut_function(gt_content, func_name, fname)
            if mutated_content is None:
                continue
            p.write_text(mutated_content, encoding="utf-8")
            did_mutate = True
        if did_mutate:
            mutated.append(fname)
        else:
            not_applicable.append(fname)
    return tmp_dir, mutated, not_applicable


def run_mutation_check(task_dir: Path):
    if not is_groundtruth_verified(task_dir):
        return {"status": "NO_GROUNDTRUTH"}
    if not (task_dir / "Dockerfile_real").exists():
        return {"status": "NO_DOCKERFILE"}

    ctx_dir, mutated, not_applicable = build_mutated_context(task_dir)
    if not mutated:
        shutil.rmtree(ctx_dir, ignore_errors=True)
        return {"status": "MUTATION_NOT_APPLICABLE", "not_applicable_files": not_applicable}

    image_tag = f"mutate-{task_dir.name}-{uuid.uuid4().hex[:8]}"
    try:
        try:
            build = subprocess.run(
                ["docker", "build", "-t", image_tag, "-f", str(ctx_dir / "Dockerfile_real"), str(ctx_dir)],
                capture_output=True, encoding="utf-8", errors="replace", timeout=900,
            )
        except subprocess.TimeoutExpired:
            return {"status": "BUILD_TIMEOUT", "mutated_files": mutated,
                    "not_applicable_files": not_applicable}
        if build.returncode != 0:
            # gutted version didn't even compile -- inconclusive for "shadow test"
            # purposes (could be a non-void C++ function that now fails to build),
            # but NOT a false pass, so not flagged as suspicious.
            return {"status": "MUTATED_BUILD_FAILED", "mutated_files": mutated,
                    "not_applicable_files": not_applicable}
        try:
            run = subprocess.run(
                ["docker", "run", "--rm", image_tag],
                capture_output=True, encoding="utf-8", errors="replace", timeout=200,
            )
        except subprocess.TimeoutExpired:
            # gutted logic made the node hang instead of returning -- that's still
            # evidence the test DOES depend on real behavior (a shadow test would
            # not hang, it would just pass immediately), so this is not suspicious,
            # just inconclusive/flaky.
            return {"status": "RUN_TIMEOUT", "mutated_files": mutated,
                    "not_applicable_files": not_applicable}
        if run.returncode == 0:
            return {"status": "SUSPICIOUS_SHADOW_TEST", "mutated_files": mutated,
                    "not_applicable_files": not_applicable,
                    "detail": "test still PASSED after the real logic was gutted -- "
                              "it is not actually exercising this file's content"}
        else:
            return {"status": "TEST_IS_REAL", "mutated_files": mutated,
                    "not_applicable_files": not_applicable}
    finally:
        subprocess.run(["docker", "rmi", "-f", image_tag], capture_output=True)
        shutil.rmtree(ctx_dir, ignore_errors=True)


def find_all_tasks():
    tasks = []
    for root, dirs, _ in os.walk(REPO_ROOT):
        if "ros1_code" in dirs and not root.endswith("/source"):
            tasks.append(Path(root))
    return sorted(tasks)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--out", type=str, default="mutation_test_results.json")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    if args.task:
        task_dirs = [Path(args.task)]
    elif args.all:
        task_dirs = [t for t in find_all_tasks() if is_groundtruth_verified(t)]
    else:
        parser.error("pass --task <dir> or --all")
        return

    results = {}
    if args.resume and os.path.exists(args.out):
        with open(args.out, encoding="utf-8") as f:
            results = json.load(f)
        print(f"Resuming: {len(results)} already done, skipping those")
        task_dirs = [t for t in task_dirs if str(t) not in results]
    for i, t in enumerate(task_dirs):
        print(f"[{i+1}/{len(task_dirs)}] {t} ...", end=" ", flush=True)
        r = run_mutation_check(t)
        print(r["status"])
        results[str(t)] = r
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)

    from collections import Counter
    c = Counter(r["status"] for r in results.values())
    print("\n=== Summary ===")
    for status, count in c.items():
        print(f"  {status}: {count}")
    suspicious = [t for t, r in results.items() if r["status"] == "SUSPICIOUS_SHADOW_TEST"]
    if suspicious:
        print(f"\n{len(suspicious)} task(s) need human review (test didn't notice gutted logic):")
        for t in suspicious:
            print("  ", t)


if __name__ == "__main__":
    main()
