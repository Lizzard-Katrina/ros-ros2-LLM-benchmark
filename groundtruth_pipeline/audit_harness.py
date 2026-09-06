#!/usr/bin/env python3
"""Sanity-checks tests/test_runtime_ros2.py for every GROUNDTRUTH_VERIFIED task:
does the harness actually reference the translated target file(s) by name, or
does it look like it reimplemented the target's logic inline instead of really
exercising it? This is a weak static proxy (substring match), not a proof of
correctness -- a task passing this check can still have a harness that doesn't
meaningfully test the target file. Use it to prioritize what to look at, not as
a final verdict.

Usage: python3 groundtruth_pipeline/audit_harness.py
"""

import json
import os
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CODE_EXT = (".py", ".cpp", ".hpp", ".h", ".cc")


def find_all_tasks():
    tasks = []
    for root, dirs, _ in os.walk(REPO_ROOT):
        if "ros1_code" in dirs and not root.endswith("/source"):
            tasks.append(Path(root))
    return sorted(tasks)


def audit():
    clean, suspicious, no_harness = [], [], []

    for t in find_all_tasks():
        status_path = t / "ros2_code" / "GROUNDTRUTH_STATUS.json"
        if not status_path.exists():
            continue
        try:
            st = json.loads(status_path.read_text())
        except Exception:
            continue
        if st.get("status") != "GROUNDTRUTH_VERIFIED":
            continue

        harness_path = t / "tests" / "test_runtime_ros2.py"
        if not harness_path.exists():
            no_harness.append(str(t.relative_to(REPO_ROOT)))
            continue
        harness = harness_path.read_text(encoding="utf-8", errors="ignore")

        ros1_dir = t / "ros1_code"
        targets = [f for f in os.listdir(ros1_dir) if f.endswith(CODE_EXT) and (ros1_dir / f).is_file()]
        target_stems = [os.path.splitext(f)[0] for f in targets]

        referenced = any(re.search(rf'\b{re.escape(stem)}\b', harness) for stem in target_stems)
        rel = str(t.relative_to(REPO_ROOT))
        (clean if referenced else suspicious).append(rel)

    return {"clean": clean, "suspicious": suspicious, "no_harness": no_harness}


if __name__ == "__main__":
    result = audit()
    out_path = Path(__file__).resolve().parent / "harness_audit.json"
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print(f"Verified tasks with harness that references target file: {len(result['clean'])}")
    print(f"SUSPICIOUS (harness never mentions target file by name):  {len(result['suspicious'])}")
    for s in result["suspicious"]:
        print("   ", s)
    print(f"No harness file at all:                                    {len(result['no_harness'])}")
    print(f"\nWrote {out_path}")
