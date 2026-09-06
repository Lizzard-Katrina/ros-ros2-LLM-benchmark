"""Best-effort function-level extraction/splicing, used for the diagnostic
"did they get just the hollowed logic right" metric (as opposed to the whole-file
metric evaluate_candidate.py uses by default).

Approach:
1. Find the TODO...END OF TODO region in ros1_code/<file> (already-hollowed file).
2. Identify the enclosing function/method that contains that region.
3. Extract that SAME-NAMED function's body from both groundtruth's file and the
   candidate's file.
4. Produce a spliced file = groundtruth's file, with only that function's body
   replaced by the candidate's version of it.

Python uses `ast` (robust). C++ uses brace-matching on a located function
signature line (best-effort -- doesn't handle every possible macro/template
edge case, but works for the straightforward member-function style used
throughout this benchmark).
"""

import re

TODO_RE = re.compile(r"(?://|#|/\*|\*)\s*TODO\b", re.I)
END_TODO_RE = re.compile(r"end.*todo|endof", re.I)


def find_todo_line(content: str) -> int:
    lines = content.splitlines()
    for i, line in enumerate(lines):
        if TODO_RE.search(line):
            return i
    return -1


# ---------- Python ----------
# Indentation-based, not ast-based: ros1_code is frequently Python 2 syntax
# (`except X, e:`, `print "..."`) or has a dangling/incomplete body around the
# TODO, either of which makes ast.parse raise SyntaxError. Scanning for a
# `def name(...):` line with less indentation than the TODO, then taking
# everything until the next line at that same (or lower) indentation, is more
# robust to both cases.

PY_DEF_RE = re.compile(r"^(\s*)(?:async\s+)?def\s+(\w+)\s*\(")


def _indent_of(line: str) -> int:
    return len(line) - len(line.lstrip(" \t"))


def _py_function_bounds_by_name_near(lines, def_line_idx):
    def_indent = _indent_of(lines[def_line_idx])
    end = len(lines) - 1
    for j in range(def_line_idx + 1, len(lines)):
        stripped = lines[j].strip()
        if stripped == "":
            continue
        if _indent_of(lines[j]) <= def_indent:
            end = j - 1
            break
    return end


def _py_enclosing_function(content: str, todo_line: int):
    lines = content.splitlines()
    for i in range(todo_line, -1, -1):
        m = PY_DEF_RE.match(lines[i])
        if not m:
            continue
        end = _py_function_bounds_by_name_near(lines, i)
        if i <= todo_line <= end:
            return m.group(2), i, end
    return None


def _py_extract_function(content: str, func_name: str):
    lines = content.splitlines()
    for i, line in enumerate(lines):
        m = PY_DEF_RE.match(line)
        if m and m.group(2) == func_name:
            end = _py_function_bounds_by_name_near(lines, i)
            return i, end, "\n".join(lines[i:end + 1])
    return None


def _py_splice(groundtruth_content: str, func_name: str, replacement_body: str):
    loc = _py_extract_function(groundtruth_content, func_name)
    if loc is None:
        return None
    start, end, _ = loc
    lines = groundtruth_content.splitlines()
    new_lines = lines[:start] + replacement_body.splitlines() + lines[end + 1:]
    return "\n".join(new_lines)


# ---------- C++ (brace-matching, best-effort) ----------

CPP_FUNC_SIG_RE = re.compile(
    r"^[\w:&\*<>,\s]+?\b(\w+)\s*\([^;{]*\)\s*(?:const\s*)?\{?\s*$"
)

CPP_CONTROL_KEYWORDS = {
    "if", "for", "while", "switch", "catch", "else", "do", "return",
    "sizeof", "static_cast", "dynamic_cast", "reinterpret_cast", "const_cast",
}


def _cpp_enclosing_function(content: str, todo_line: int):
    lines = content.splitlines()
    # scan backward from todo_line for a line that looks like a function
    # signature AND whose brace-matched body actually contains todo_line
    for i in range(todo_line, -1, -1):
        stripped = lines[i].strip()
        m = CPP_FUNC_SIG_RE.match(stripped)
        if not m or m.group(1) in CPP_CONTROL_KEYWORDS:
            continue
        # require the line to actually start with the captured token (or a
        # qualified/typed prefix ending right before it) -- rules out partial
        # matches like "if(...)" being split into a bogus prefix+name
        if not re.match(r"^[\w:&\*<>,\s]*\b" + re.escape(m.group(1)) + r"\s*\(", stripped):
            continue
        # find the opening brace at/after this line
        brace_start = None
        for j in range(i, min(i + 3, len(lines))):
            if "{" in lines[j]:
                brace_start = j
                break
        if brace_start is None:
            continue
        end = _match_closing_brace(lines, brace_start)
        if end is not None and brace_start <= todo_line <= end:
            return m.group(1), i, end
    return None


def _match_closing_brace(lines, brace_start_line):
    depth = 0
    started = False
    for j in range(brace_start_line, len(lines)):
        for ch in lines[j]:
            if ch == "{":
                depth += 1
                started = True
            elif ch == "}":
                depth -= 1
        if started and depth == 0:
            return j
    return None


def _cpp_extract_function(content: str, func_name: str):
    lines = content.splitlines()
    for i, line in enumerate(lines):
        m = CPP_FUNC_SIG_RE.match(line.strip())
        if not m or m.group(1) != func_name:
            continue
        brace_start = None
        for j in range(i, min(i + 3, len(lines))):
            if "{" in lines[j]:
                brace_start = j
                break
        if brace_start is None:
            continue
        end = _match_closing_brace(lines, brace_start)
        if end is None:
            continue
        return i, end, "\n".join(lines[i:end + 1])
    return None


def _cpp_splice(groundtruth_content: str, func_name: str, replacement_body: str):
    loc = _cpp_extract_function(groundtruth_content, func_name)
    if loc is None:
        return None
    start, end, _ = loc
    lines = groundtruth_content.splitlines()
    new_lines = lines[:start] + replacement_body.splitlines() + lines[end + 1:]
    return "\n".join(new_lines)


# ---------- public API ----------

def is_cpp(fname: str) -> bool:
    return fname.endswith((".cpp", ".hpp", ".h", ".cc"))


def find_hollowed_function_name(ros1_content: str, fname: str):
    todo_line = find_todo_line(ros1_content)
    if todo_line == -1:
        return None
    if is_cpp(fname):
        found = _cpp_enclosing_function(ros1_content, todo_line)
    else:
        found = _py_enclosing_function(ros1_content, todo_line)
    return found[0] if found else None


def splice_function_into_groundtruth(groundtruth_content: str, candidate_content: str,
                                      func_name: str, fname: str):
    """Returns groundtruth_content with func_name's body replaced by candidate's
    version of that same function, or None if extraction failed on either side."""
    if is_cpp(fname):
        cand_loc = _cpp_extract_function(candidate_content, func_name)
        if cand_loc is None:
            return None
        return _cpp_splice(groundtruth_content, func_name, cand_loc[2])
    else:
        cand_loc = _py_extract_function(candidate_content, func_name)
        if cand_loc is None:
            return None
        return _py_splice(groundtruth_content, func_name, cand_loc[2])
