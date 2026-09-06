#!/usr/bin/env python3
"""Scores every model's translations (across results/results1..5) against
groundtruth, using evaluate_candidate.py's whole-file metric (primary) and
optionally the function-splice diagnostic. Scans:
    <level>/results{N}/<model>/<category>/<task_name>/dialogue.md
and matches each to the corresponding groundtruth task at
    <level>/<category>/<task_name>/

Usage:
    python3 groundtruth_pipeline/score_benchmark.py                    # everything found
    python3 groundtruth_pipeline/score_benchmark.py --limit 5          # smoke test
    python3 groundtruth_pipeline/score_benchmark.py --model openai_gpt-5.5
    python3 groundtruth_pipeline/score_benchmark.py --results-run results1
    python3 groundtruth_pipeline/score_benchmark.py --with-function-diagnostic
"""

import argparse
import json
import os
import re
from collections import defaultdict
from pathlib import Path

import sys
sys.path.insert(0, os.path.dirname(__file__))
from evaluate_candidate import evaluate_task, evaluate_task_function_level, is_groundtruth_verified

REPO_ROOT = Path(__file__).resolve().parent.parent
LEVELS = ["interface_level", "behavior_level", "system_level"]
RESULTS_DIR_RE = re.compile(r"^results\d*$")


def find_dialogues(model_filter=None, results_run_filter=None):
    """Yields (level, results_dir_name, model, category, task_name, dialogue_path, groundtruth_task_dir)."""
    for level in LEVELS:
        level_dir = REPO_ROOT / level
        if not level_dir.is_dir():
            continue
        for results_dir in sorted(level_dir.iterdir()):
            if not results_dir.is_dir() or not RESULTS_DIR_RE.match(results_dir.name):
                continue
            if results_run_filter and results_dir.name != results_run_filter:
                continue
            for model_dir in sorted(results_dir.iterdir()):
                if not model_dir.is_dir():
                    continue
                if model_filter and model_dir.name != model_filter:
                    continue
                for category_dir in sorted(model_dir.iterdir()):
                    if not category_dir.is_dir():
                        continue
                    for task_dir in sorted(category_dir.iterdir()):
                        dialogue = task_dir / "dialogue.md"
                        if not dialogue.exists():
                            continue
                        gt_task_dir = level_dir / category_dir.name / task_dir.name
                        yield (level, results_dir.name, model_dir.name, category_dir.name,
                               task_dir.name, dialogue, gt_task_dir)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default=None, help="only this model dir name, e.g. openai_gpt-5.5")
    parser.add_argument("--results-run", type=str, default=None, help="only this results dir, e.g. results1")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--with-function-diagnostic", action="store_true")
    parser.add_argument("--out", type=str, default="benchmark_scores.json")
    parser.add_argument("--resume", action="store_true",
                         help="load --out if it exists and skip (model, results_run, category, task) "
                              "combinations already recorded in it")
    args = parser.parse_args()

    entries = list(find_dialogues(args.model, args.results_run))
    if args.limit:
        entries = entries[: args.limit]

    results = []
    done_keys = set()
    if args.resume and os.path.exists(args.out):
        with open(args.out, encoding="utf-8") as f:
            prior = json.load(f)
        results = prior.get("results", [])
        done_keys = {(r["model"], r["results_run"], r["category"], r["task"]) for r in results}
        print(f"Resuming: {len(done_keys)} combinations already scored in {args.out}, will skip those")
        entries = [e for e in entries if (e[2], e[1], e[3], e[4]) not in done_keys]

    print(f"{len(entries)} (model, run, task) combinations left to score")

    scoreboard = defaultdict(lambda: {"pass": 0, "fail": 0, "skipped": 0})
    for r in results:
        key = f"{r['model']} / {r['results_run']}"
        if r["status"] == "PASS":
            scoreboard[key]["pass"] += 1
        elif r["status"] == "FAIL":
            scoreboard[key]["fail"] += 1
        else:
            scoreboard[key]["skipped"] += 1

    for i, (level, run, model, category, task_name, dialogue, gt_task_dir) in enumerate(entries):
        key = f"{model} / {run}"
        label = f"[{i+1}/{len(entries)}] {key} :: {category}/{task_name}"
        if not gt_task_dir.exists() or not is_groundtruth_verified(gt_task_dir):
            print(f"{label} -> SKIP (no verified groundtruth)")
            scoreboard[key]["skipped"] += 1
            continue

        print(label, end=" ... ", flush=True)
        try:
            outcome = evaluate_task(gt_task_dir, dialogue_md=dialogue, verbose=False)
            status = outcome.get("status")
            print(status)

            entry = {
                "model": model, "results_run": run, "level": level,
                "category": category, "task": task_name,
                "status": status, "stage_failed": outcome.get("stage_failed"),
            }
            if args.with_function_diagnostic and status != "NO_CANDIDATE_FILES":
                try:
                    fdiag = evaluate_task_function_level(gt_task_dir, dialogue_md=dialogue, verbose=False)
                    entry["function_level_status"] = fdiag.get("status")
                except Exception as e:
                    entry["function_level_status"] = f"DIAGNOSTIC_ERROR: {e}"
        except Exception as e:
            print(f"ERROR: {e}")
            status = "ERROR"
            entry = {
                "model": model, "results_run": run, "level": level,
                "category": category, "task": task_name,
                "status": status, "stage_failed": None, "error": str(e),
            }

        results.append(entry)
        if status == "PASS":
            scoreboard[key]["pass"] += 1
        elif status in ("FAIL",):
            scoreboard[key]["fail"] += 1
        else:
            scoreboard[key]["skipped"] += 1

        # save after every entry so a crash/kill never loses more than the
        # one in-flight docker build+run -- --resume picks up from here
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"results": results, "scoreboard": dict(scoreboard)}, f, indent=2)

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"results": results, "scoreboard": dict(scoreboard)}, f, indent=2)

    print("\n=== Scoreboard ===")
    for key, s in sorted(scoreboard.items()):
        total = s["pass"] + s["fail"]
        rate = f"{s['pass']}/{total}" if total else "0/0"
        print(f"  {key}: {rate} passed ({s['skipped']} skipped)")
    print(f"\nFull results written to {args.out}")


if __name__ == "__main__":
    main()
