import re
import pytest
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "msg2fbs.py"
FBS_FILE = Path(__file__).resolve().parents[1] / "schema.fbs"

def get_content(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# --- 1. Cross-Language Interface Synchronization ---

def test_time_struct_field_sync():
    """Verify field names match between Python generator and static FBS schema."""
    py_content = get_content(PY_FILE)
    fbs_content = get_content(FBS_FILE)
    
    # Check if Python generates 'sec' (ROS 2) or 'secs' (ROS 1)
    py_has_sec = "sec:" in py_content and "nanosec:" in py_content
    # Check if FBS uses 'sec' or 'secs'
    fbs_has_sec = re.search(r"struct\s+RosTime\s*\{[^}]*sec:uint32", fbs_content)
    
    assert py_has_sec, "Python generator must use ROS 2 'sec/nanosec' fields."
    assert fbs_has_sec, "FBS Schema must use ROS 2 'sec' field to maintain binary compatibility."

# --- 2. ROS 2 Structural Correctness ---

def test_header_schema_no_seq():
    """Confirm the Header table is correctly refactored for ROS 2 (removed seq)."""
    fbs_content = get_content(FBS_FILE)
    
    header_block = re.search(r"table\s+Header\s*\{([\s\S]*?)\}", fbs_content)
    assert header_block, "Header table not found in schema.fbs"
    
    content = header_block.group(1)
    assert "seq" not in content, "ROS 2 Header should not contain a 'seq' field."
    assert "stamp:RosTime" in content or "stamp:fb.RosTime" in content, "Header must contain a 'stamp' of type RosTime."

def test_namespace_conversion_logic():
    """Check if Python logic correctly handles ROS 2 '/' to FBS '.' conversion."""
    py_content = get_content(PY_FILE)
    
    # Look for the logic that handles ROS 2 'pkg/msg/Type' -> 'pkg.Type'
    assert ".replace(\"/\", \".\")" in py_content or ".replace('/', '.')" in py_content, \
        "The Type class must convert ROS paths to dot-separated namespaces."

# --- 3. Anti-Regression / Safety ---

def test_no_ros1_naming_leakage():
    """Ensure no 'secs' or 'nsecs' (ROS 1) strings exist in the refactored core."""
    combined = get_content(PY_FILE) + get_content(FBS_FILE)
    
    # Using word boundaries to avoid matching things like 'seconds'
    assert not re.search(r"\bsecs\b", combined), "Detected ROS 1 naming convention leakage ('secs')."
    assert not re.search(r"\bnsecs\b", combined), "Detected ROS 1 naming convention leakage ('nsecs')."

def test_metadata_inclusion():
    """Verify that the metadata fields are still generated in the new logic."""
    py_content = get_content(PY_FILE)
    assert "__metadata" in py_content, "Python generator lost the __metadata field requirement."
