#!/usr/bin/env python3
"""Extracts clean {filename: content} candidate files out of a run_all.py /
run_all_5.py dialogue.md (which mixes the prompt and the LLM's raw response,
using the same `[FILENAME: x]` marker convention those scripts already use).

Usage as a library:
    from extract_candidate import extract_files_from_dialogue
    files = extract_files_from_dialogue(Path(".../dialogue.md"))

CLI:
    python3 groundtruth_pipeline/extract_candidate.py path/to/dialogue.md
"""

import re
import sys
from pathlib import Path

RESPONSE_SPLIT_RE = re.compile(r"^# LLM Response\s*$", re.MULTILINE)
FILENAME_SPLIT_RE = re.compile(r"\[FILENAME:\s*(.*?)\]")


def extract_files_from_dialogue(dialogue_path: Path, fallback_filename: str = None) -> dict:
    text = dialogue_path.read_text(encoding="utf-8", errors="ignore")
    parts = RESPONSE_SPLIT_RE.split(text, maxsplit=1)
    response = parts[1] if len(parts) > 1 else text

    chunks = FILENAME_SPLIT_RE.split(response)
    files = {}
    for i in range(1, len(chunks), 2):
        fname = chunks[i].strip()
        content = chunks[i + 1].strip()
        content = re.sub(r"^```[a-zA-Z0-9_+-]*\n", "", content)
        content = re.sub(r"\n```$", "", content)
        # matches run_all_5.py's own convention of stripping a "_todo" suffix
        fname = fname.replace("_todo", "")
        files[fname] = content

    if not files and fallback_filename:
        # older/simpler dialogue.md format (single-file tasks): no [FILENAME: ..]
        # marker at all, the whole response IS the one file's content.
        content = response.strip()
        content = re.sub(r"^```[a-zA-Z0-9_+-]*\n", "", content)
        content = re.sub(r"\n```$", "", content)
        if content:
            files[fallback_filename] = content
    return files


def materialize(files: dict, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for fname, content in files.items():
        dest = out_dir / fname
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(content, encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: extract_candidate.py path/to/dialogue.md")
        sys.exit(1)
    files = extract_files_from_dialogue(Path(sys.argv[1]))
    for fname, content in files.items():
        print(f"--- {fname} ({len(content)} chars) ---")
