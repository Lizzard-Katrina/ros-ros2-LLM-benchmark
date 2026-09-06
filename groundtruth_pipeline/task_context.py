"""Gathers everything the LLM needs to know about a single task."""

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

CODE_EXT = (".py", ".cpp", ".hpp", ".h", ".cc")


@dataclass
class TaskContext:
    task_dir: Path
    metadata: dict = field(default_factory=dict)
    readme: str = ""
    ros1_files: dict = field(default_factory=dict)          # filename -> content (hollowed, has TODO)
    reference_source_files: dict = field(default_factory=dict)  # filename -> content (cloned ROS1 upstream, no TODO)
    reference_notes: str = ""  # informal reference (e.g. fetched tutorial page text), not a diffable source file
    oracle_test: str = ""
    language: str = "python"

    @property
    def name(self) -> str:
        return str(self.task_dir)


def load_task_context(task_dir: str) -> TaskContext:
    task_dir = Path(task_dir)
    ctx = TaskContext(task_dir=task_dir)

    meta_path = task_dir / "metadata.json"
    if meta_path.exists():
        try:
            ctx.metadata = json.loads(meta_path.read_text(encoding="utf-8", errors="ignore"))
        except Exception:
            ctx.metadata = {}

    readme_path = task_dir / "README.md"
    if readme_path.exists():
        ctx.readme = readme_path.read_text(encoding="utf-8", errors="ignore")

    ros1_dir = task_dir / "ros1_code"
    for f in sorted(os.listdir(ros1_dir)):
        fp = ros1_dir / f
        if fp.is_file() and f.endswith(CODE_EXT):
            ctx.ros1_files[f] = fp.read_text(encoding="utf-8", errors="ignore")
            if f.endswith((".cpp", ".hpp", ".h", ".cc")):
                ctx.language = "cpp"

    source_dir = ros1_dir / "source"
    if source_dir.exists():
        notes_path = source_dir / "REFERENCE_NOTES.md"
        if notes_path.exists():
            ctx.reference_notes = notes_path.read_text(encoding="utf-8", errors="ignore")
        for root, _, files in os.walk(source_dir):
            for f in files:
                if f in ("SOURCE_INFO.json", "REFERENCE_NOTES.md") or not f.endswith(CODE_EXT + (".srv", ".msg", ".action")):
                    continue
                fp = Path(root) / f
                rel = str(fp.relative_to(source_dir))
                ctx.reference_source_files[rel] = fp.read_text(encoding="utf-8", errors="ignore")

    oracle_path = task_dir / "tests" / "test_oracle_ros2.py"
    if oracle_path.exists():
        ctx.oracle_test = oracle_path.read_text(encoding="utf-8", errors="ignore")

    return ctx


def has_unresolved_todo(content: str) -> bool:
    return bool(re.search(r"\bTODO\b", content, re.I))
