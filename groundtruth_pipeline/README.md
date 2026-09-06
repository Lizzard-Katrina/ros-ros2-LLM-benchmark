# Groundtruth Pipeline

## Goal

The benchmark's existing `tests/test_oracle_ros2.py` files are static regex checks on
source text -- they never actually run the translated ROS2 code. This pipeline exists
to produce **groundtruth ROS2 translations that are proven to actually build and run**
in a real Docker/ROS2 Humble environment, not just pass a text pattern.

A task is only considered done when it reaches **Tier3**:

| Tier | Check | Where |
|---|---|---|
| Tier1 | `colcon build` succeeds in Docker | `docker_verify.py` |
| Tier2 | existing static oracle regex tests pass | `tests/test_oracle_ros2.py` (unchanged, kept as a secondary signal) |
| Tier3 | a real runtime test launches the translated node and asserts on **actual** returned data (pub/sub payload, service response, param value, etc.) | `docker_verify.py` runs `tests/test_runtime_ros2.py` inside the container |

`ros2_code/GROUNDTRUTH_STATUS.json` only gets `"status": "GROUNDTRUTH_VERIFIED"` when
Tier1 AND Tier3 both pass for real. Tier1-only ("it compiled") is explicitly **not**
good enough and is recorded as `GROUNDTRUTH_UNVERIFIED_MAX_RETRIES` instead.

## Code structure

```
groundtruth_pipeline/
  task_context.py       -- reads ros1_code/*.py|cpp (with TODOs), ros1_code/source/*
                            (cloned original ROS1 repo, no TODOs), metadata.json,
                            README.md, tests/test_oracle_ros2.py for one task
  prompts.py             -- SYSTEM_PROMPT + prompt builders + [FILENAME: ...] block parser
  openrouter_client.py   -- thin OpenRouter chat-completion wrapper (same pattern as
                            pipeline.py / run_all_5.py elsewhere in this repo)
  docker_verify.py       -- spins up a real `osrf/ros:humble-desktop` container, copies
                            the candidate package in, runs colcon build, then runs
                            test_runtime_ros2.py inside it
  build_groundtruth.py   -- orchestrator: prompt -> LLM -> parse files -> docker verify
                            -> on failure, feed the build/test log back to the LLM and
                            retry (--max-fix-rounds, default 4) -> persist result
  task_manifest.json     -- generated list of all 108 benchmark tasks, each tagged
                            eligible/not-eligible with a reason (regenerate with
                            `python3 build_manifest.py`, see below)
```

### What gets persisted per task (once verified)

- `ros2_code/source/` -- the translated node file(s) **and** the full buildable package
  skeleton (`package.xml` + `setup.py`/`setup.cfg` or `CMakeLists.txt`, plus any custom
  `.srv`/`.msg` the LLM had to add)
- `tests/test_runtime_ros2.py` -- the Tier3 harness, kept permanently so the check is
  re-runnable later (e.g. against a different model's submission), not a throwaway
- `ros2_code/GROUNDTRUTH_STATUS.json` -- final status + per-round build/test outcome log

## Model

`openrouter_client.DEFAULT_MODEL` is currently `"anthropic/claude-opus-4.6"`, carried
over from the model list already used in `run_all_5.py`. **This was not a deliberate
choice for groundtruth quality** -- override it explicitly with `--model`. Two things
worth deciding before a big batch run:

1. Pick the strongest model you trust, since this becomes the answer key.
2. Prefer a model that is **not** also in your benchmark's `MODELS_TO_TEST` list, to
   avoid using a model to grade itself.

## Which tasks can run through this pipeline

Eligibility is computed from whether `ros1_code/source/` was successfully cloned and
not flagged `NEEDS_MANUAL_REVIEW` (see the earlier clone-triage pass). Current snapshot
(regenerate anytime, see below):

- **85 / 108 tasks eligible** (have a real cloned ROS1 reference under `ros1_code/source/`)
  - 71 not started (`GROUNDTRUTH_STATUS.json` absent) -- these are what `--all --skip-existing` will process
  - 1 verified through this exact pipeline end-to-end so far: none yet automated (task_003 was done manually pre-pipeline, see its status note)
  - 7 previously marked `DIRECT_EXTRACT_VERIFIED` -- **text-diff verified only, NOT yet Tier3 docker-verified** -- still need a pipeline pass
- **23 / 108 tasks NOT eligible**, split into two reasons:
  - **16**: no usable source link at all in `metadata.json`/`README.md` (tutorial-style tasks, or link genuinely missing)
  - **7**: a link exists but resolves to something wrong (a GitHub Gist instead of a repo, or a repo/branch where the claimed filename doesn't actually exist) -- flagged `NEEDS_MANUAL_REVIEW`, need a corrected source link from you before they're eligible

Full per-task list with reasons: `task_manifest.json` (one row per task, `eligible: true/false`, `reason`, `current_groundtruth_status`).

## Usage

```bash
export OPENROUTER_API_KEY=...
# single task, useful to sanity-check the pipeline before a big batch
python3 groundtruth_pipeline/build_groundtruth.py --task interface_level/service_client/task_003_mp3_db_service --model <your-chosen-model>

# small batch first
python3 groundtruth_pipeline/build_groundtruth.py --all --limit 5 --model <your-chosen-model>

# full run, resumable
python3 groundtruth_pipeline/build_groundtruth.py --all --skip-existing --model <your-chosen-model>
```

This has **not yet been run end-to-end against a real model** in this session (no
OpenRouter key available here) -- the non-LLM parts (task scanning, eligibility,
prompt assembly, file-block parsing, Docker build/run mechanics) were unit-tested
directly and work; the LLM translation + auto-fix-retry loop itself is unverified

