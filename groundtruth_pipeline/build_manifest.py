#!/usr/bin/env python3
"""Regenerates task_manifest.json: every benchmark task tagged eligible/not-eligible
for the groundtruth pipeline, with a reason, plus its current groundtruth status."""

import json
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def find_all_tasks():
    tasks = []
    for root, dirs, _ in os.walk(REPO_ROOT):
        if "ros1_code" in dirs and not root.endswith("/source"):
            tasks.append(Path(root))
    return sorted(tasks)


def build():
    manifest = []
    for t in find_all_tasks():
        rel = str(t.relative_to(REPO_ROOT))
        source_dir = t / "ros1_code" / "source"
        info_path = source_dir / "SOURCE_INFO.json"
        gt_status_path = t / "ros2_code" / "GROUNDTRUTH_STATUS.json"
        gt_info_path = t / "ros2_code" / "source" / "GROUNDTRUTH_INFO.json"

        entry = {"task": rel}
        if not source_dir.is_dir():
            # no cloned reference at all -- still processable (translate directly
            # from the hollowed ros1_code), matches build_groundtruth.is_eligible()
            entry["eligible"] = True
            entry["reason"] = "no ros1_code/source cloned (no reference material) -- will translate from ros1_code alone"
        elif info_path.exists():
            try:
                info = json.loads(info_path.read_text())
            except Exception:
                info = {}
            if info.get("status") == "NEEDS_MANUAL_REVIEW":
                entry["eligible"] = False
                entry["reason"] = info.get("reason", "flagged NEEDS_MANUAL_REVIEW")
            else:
                entry["eligible"] = True
                entry["reason"] = None
        else:
            entry["eligible"] = True
            entry["reason"] = None

        if entry["eligible"]:
            if gt_status_path.exists():
                try:
                    entry["current_groundtruth_status"] = json.loads(gt_status_path.read_text()).get("status", "UNKNOWN")
                except Exception:
                    entry["current_groundtruth_status"] = "UNKNOWN"
            elif gt_info_path.exists():
                entry["current_groundtruth_status"] = "DIRECT_EXTRACT_VERIFIED (text-diff only, NOT yet Tier3 docker-verified)"
            else:
                entry["current_groundtruth_status"] = "NOT_STARTED"
        else:
            entry["current_groundtruth_status"] = "N/A"

        manifest.append(entry)
    return manifest


if __name__ == "__main__":
    manifest = build()
    out_path = Path(__file__).resolve().parent / "task_manifest.json"
    out_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    eligible = [m for m in manifest if m["eligible"]]
    print(f"Total: {len(manifest)}  Eligible: {len(eligible)}  Not eligible: {len(manifest) - len(eligible)}")
    print(f"Wrote {out_path}")
