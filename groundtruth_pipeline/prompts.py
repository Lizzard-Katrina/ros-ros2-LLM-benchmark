"""Prompt construction + [FILENAME: ...] block parsing for the groundtruth pipeline."""

import re

SYSTEM_PROMPT = """You are an expert ROS1-to-ROS2 (Humble) migration engineer building a
VERIFIED GROUND TRUTH reference solution for a robotics benchmark. This is not a
demo -- your output will be compiled with colcon and actually executed in a
real ROS2 Docker container, then exercised with a live runtime test.

Hard requirements:
1. Fill in every TODO in the given ROS1 file(s), producing complete, idiomatic ROS2 rclpy/rclcpp code.
   Keep filenames, class names, function names and topic/service/action names IDENTICAL to the
   original file, unless they are ROS1-specific APIs that must change.
2. Produce a MINIMAL, BUILDABLE ament package around that code:
   - package.xml (format 3)
   - for Python: setup.py + setup.cfg (script-dir must be $base/lib/<pkg>) + an empty
     resource/<pkg> marker file + <pkg>/__init__.py
   - for C++: CMakeLists.txt
   Only depend on packages available via `apt install ros-humble-*` on Humble -- do not invent
   packages that don't exist. If a custom .srv/.msg/.action is needed, include it under srv/msg/action
   and reference it correctly from package.xml/CMakeLists.txt.
3. Produce ONE runtime test file, at the EXACT path `test_runtime_ros2.py` (package root,
   no subdirectory -- not `test/test_runtime_ros2.py`, not `tests/test_runtime_ros2.py`).
   Do NOT wire it into CMakeLists.txt/ament_add_pytest_test or colcon test in any way --
   it is run directly with `pytest test_runtime_ros2.py` by an external harness, completely
   independent of the colcon build. CMakeLists.txt/setup.py must not reference this file at
   all. It (pytest, using rclpy) must:
   - Actually exercise the translated file(s) you just wrote (same filenames as the
     ros1_code files you translated) -- either `subprocess.Popen(["ros2", "run", <pkg>,
     <executable-name-that-runs-that-file>, ...])`, or `from <pkg>.<module> import <Class>`
     and instantiate/call it directly in-process. CRITICAL: you must NOT reimplement or
     duplicate the target file's logic inline inside the test (e.g. writing your own copy
     of the node/server as a string passed to `python3 -c "..."`, or hand-rolling equivalent
     behavior in the test itself). If the test would still pass after the translated file's
     source is replaced with garbage, it is invalid -- it must import or launch that exact
     file/executable by name, so a wrong translation actually makes the test fail.
   - If a counterpart node is needed to interact with the translated node (e.g. a mock
     service server so a translated service *client* has something to call, or a publisher
     so a translated *subscriber* has something to receive), write that counterpart as a
     SEPARATE helper file (e.g. `_test_helper_node.py`), not inlined as a string in the test,
     and launch it the same way. Only the translated file(s) may be swapped out later by a
     different candidate submission -- everything else (this helper included) must stay
     reusable as-is, so keep the translated file(s) and the test helper cleanly separate.
   - Uses a SEPARATE rclpy node created inside the test to perform a REAL interaction
     (publish and check a real subscriber callback fired with expected data, OR call a
     real service/action and assert on the actual response content, OR read back a real
     parameter value). Do not just assert "no exception" -- assert on concrete expected
     values/content, matching the semantics of the original ROS1 code.
   - Has explicit timeouts (a few seconds) so it can never hang forever.
   - Cleans up (destroys nodes, terminates subprocesses) in a finally block.
   - Exits with a normal pytest pass/fail signal.
   - If the node depends on real hardware (serial port, camera, IMU, etc.) that can't
     exist in Docker: still actually run/import the real translated node, but fake ONLY
     the hardware boundary (e.g. a pty/socat virtual serial pair, a fake publisher on the
     raw sensor topic the node subscribes to, monkeypatching just the driver open() call)
     -- then drive it with real input and assert on its real output. Do NOT fall back to
     regex/text-parsing the source file instead of running it; that is a static check, not
     a runtime one, and defeats the entire point of this test.
4. You are given (a) the original un-hollowed ROS1 reference source (when available) -- use it to
   understand the exact intended behavior, and (b) the existing static "oracle" test for this task --
   your code must also satisfy those regex-based checks (matching names/APIs they look for), since
   they will keep running alongside your runtime test.
5. Do not explain anything. Output ONLY file blocks in this exact format, one per file:

[FILENAME: path/relative/to/package/root]
<full file content, no markdown fences>
[END FILENAME]

Always include package.xml and the build file (setup.py+setup.cfg, or CMakeLists.txt) and
test_runtime_ros2.py in every response, even on a fix-retry.
"""


def build_initial_prompt(ctx) -> str:
    parts = [
        f"Task category/hierarchy: {ctx.task_dir}",
        f"Task metadata: {ctx.metadata}",
    ]
    if ctx.readme:
        parts.append(f"--- README ---\n{ctx.readme[:4000]}")

    parts.append("--- ROS1 file(s) to translate (contain TODO markers) ---")
    for fname, content in ctx.ros1_files.items():
        parts.append(f"FILE: {fname}\n{content}")

    if ctx.reference_source_files:
        parts.append("--- Original un-hollowed ROS1 reference source (ground truth of intended logic) ---")
        for fname, content in ctx.reference_source_files.items():
            parts.append(f"FILE: {fname}\n{content[:6000]}")

    if ctx.reference_notes:
        parts.append(
            "--- Informal reference notes (fetched from a tutorial page, NOT a literal source file -- "
            "use only as conceptual/API guidance, not something to copy verbatim) ---\n"
            f"{ctx.reference_notes[:4000]}"
        )

    if ctx.oracle_test:
        parts.append(f"--- Existing static oracle test (must keep passing) ---\n{ctx.oracle_test}")

    parts.append(
        f"\nTarget package name: {ctx.task_dir.name}\n"
        f"Language: {ctx.language}\n"
        "Now produce the full set of files as specified in the system prompt."
    )
    return "\n\n".join(parts)


def build_fix_prompt(previous_files: dict, failure_stage: str, log: str) -> str:
    files_block = "\n\n".join(
        f"[FILENAME: {name}]\n{content}\n[END FILENAME]" for name, content in previous_files.items()
    )
    return f"""Your previous submission failed at stage: {failure_stage}.

--- Failure log (truncated) ---
{log[-6000:]}

--- Your previous file set ---
{files_block}

Fix the problem. Output the COMPLETE corrected file set again using the same
[FILENAME: ...] / [END FILENAME] format for every file (not just the changed ones).
"""


FILE_BLOCK_RE = re.compile(
    r"^\[FILENAME:\s*(.+?)\]\s*$\n(.*?)(?:\n)?^\[END FILENAME\]\s*$",
    re.DOTALL | re.MULTILINE,
)


def parse_file_blocks(llm_output: str) -> dict:
    """Parses [FILENAME: x]/[END FILENAME] blocks.

    [END FILENAME] must be anchored to the start of its own line (MULTILINE),
    so it can never be swallowed as "content" of an empty/near-empty file that's
    immediately followed by another block with no blank line in between -- that
    used to silently merge two files into one and corrupt the first file's content.
    """
    files = {}
    for match in FILE_BLOCK_RE.finditer(llm_output):
        fname = match.group(1).strip()
        content = match.group(2)
        content = re.sub(r"^```[a-zA-Z0-9_+-]*\n", "", content)
        content = re.sub(r"\n```$", "", content)
        files[fname] = content
    return files
