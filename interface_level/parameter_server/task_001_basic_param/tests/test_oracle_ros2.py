import re
from pathlib import Path

CLIENT_FILE = Path(__file__).resolve().parents[1] / "param.cpp"



def read_file(p: Path) -> str:
    assert p.exists(), f"Missing file: {p}"
    return p.read_text(encoding="utf-8", errors="ignore")


def extract_fn(code: str, name: str) -> str:
    m = re.search(rf"(?s)\b{name}\s*\([^)]*\)\s*\{{", code)
    assert m, f"Function '{name}' not found"
    i = m.end()
    depth = 1
    while i < len(code) and depth > 0:
        if code[i] == "{":
            depth += 1
        elif code[i] == "}":
            depth -= 1
        i += 1
    return code[m.start():i]


def test_no_ros1_master_xmlrpc():
    code = read_file(CLIENT_FILE)
    assert re.search(r"\brclcpp\b", code), "Must use ROS2 rclcpp"
    assert "ros::master::execute" not in code, "Must not use ROS1 master::execute"
    assert "XMLRPCManager" not in code, "Must not depend on ROS1 XMLRPCManager"


def test_has_cache_map_and_subscribed_set():
    code = read_file(CLIENT_FILE)

    # Cache map: key->value
    assert re.search(r"\b(std::)?(unordered_)?map\s*<\s*std::string\s*,", code), \
        "Must define a cache map keyed by std::string"

    # Subscribed set: set/unordered_set of strings
    assert re.search(r"\b(std::)?(unordered_)?set\s*<\s*std::string\s*>", code), \
        "Must define a subscribed key set (set/unordered_set<std::string>)"

    # Locking primitive
    assert re.search(r"\bstd::mutex\b|\bstd::recursive_mutex\b", code), \
        "Must use a mutex for cache/subscription state"


def test_parameter_event_subscription_is_real():
    code = read_file(CLIENT_FILE)

    # Must use ParameterEvent or ParameterEventHandler
    assert re.search(r"rcl_interfaces::msg::ParameterEvent|ParameterEventHandler", code), \
        "Must use ROS2 parameter event mechanism"

    # Must actually create a subscription or handler object (not only mention in comment)
    # subscription pattern: create_subscription<...ParameterEvent...>(
    assert re.search(r"create_subscription\s*<\s*.*ParameterEvent", code) or \
           re.search(r"ParameterEventHandler\s*::\s*SharedPtr", code) or \
           re.search(r"std::make_shared\s*<\s*.*ParameterEventHandler", code), \
        "Must create a parameter event subscription/handler object"


def test_update_is_gated_by_subscribed_set_and_mutates_cache():
    code = read_file(CLIENT_FILE)
    blk = extract_fn(code, "update")

    # Must lock in update()
    assert re.search(r"lock_guard|scoped_lock|unique_lock", blk), \
        "update must lock around shared state"

    # Must check membership in subscribed set before writing cache
    # (look for find/count + conditional)
    assert re.search(r"(subscrib\w*).*(find|count)\s*\(", blk, re.IGNORECASE), \
        "update must consult subscribed set (find/count) before cache update"
    assert re.search(r"\bif\s*\(", blk), \
        "update should gate behavior with an if()"

    # Must mutate cache (assignment/insert/erase)
    assert re.search(r"(cache|params)\w*\s*\[\s*.*\s*\]\s*=", blk) or \
           re.search(r"(cache|params)\w*\.\s*(insert|emplace|erase)\s*\(", blk), \
        "update must write into cache (assignment/insert/emplace/erase)"

    # Must call invalidateParentParams
    assert re.search(r"invalidateParentParams\s*\(", blk), \
        "update must call invalidateParentParams"


def test_invalidate_parent_walks_up_namespaces_and_erases_parents():
    code = read_file(CLIENT_FILE)
    blk = extract_fn(code, "invalidateParentParams")

    # Must lock
    assert re.search(r"lock_guard|scoped_lock|unique_lock", blk), \
        "invalidateParentParams must lock around cache"

    # Must have a loop walking parents
    assert re.search(r"\bwhile\s*\(", blk) or re.search(r"\bfor\s*\(", blk), \
        "invalidateParentParams must iterate over parent namespaces"

    # Must compute parent namespace (some parent function or string trimming)
    assert re.search(r"parent|namespace|rfind|find_last_of|substr", blk, re.IGNORECASE), \
        "invalidateParentParams must compute parent namespace keys"

    # Must erase a computed parent key from cache
    assert re.search(r"(cache|params)\w*\.\s*erase\s*\(\s*\w+", blk), \
        "invalidateParentParams must erase computed parent keys from cache"


def test_getImpl_has_cache_hit_shortcut_and_subscribe_on_miss():
    code = read_file(CLIENT_FILE)
    blk = extract_fn(code, "getImpl")

    # Must branch on use_cache
    assert "use_cache" in blk, "getImpl must branch on use_cache"

    # Must attempt cache lookup and return early on hit
    assert re.search(r"(cache|params)\w*\.\s*(find|count)\s*\(", blk), \
        "getImpl must attempt cache lookup"
    assert re.search(r"\breturn\s+true\b", blk), \
        "getImpl should have a fast-path return true on cache hit"

    # Must ensure subscription/interest tracking on first access when caching enabled
    # require insert/emplace into subscribed set or equivalent
    assert re.search(r"(subscrib\w*).*(insert|emplace)\s*\(", blk, re.IGNORECASE), \
        "getImpl must register interest in a key when caching is enabled"

    # Must have a remote query path via ROS2 parameters client APIs
    assert re.search(r"get_parameter|get_parameters|SyncParametersClient|AsyncParametersClient|set_on_parameters_set_callback|create_client", blk), \
        "getImpl must have a remote parameter query path"


def test_event_callback_calls_update_or_equivalent():
    code = read_file(CLIENT_FILE)

    # Enforce that parameter event callback leads to cache update function.
    # Look for lambda/callback that references update(...
    assert re.search(r"update\s*\(", code), \
        "Solution must route parameter events into cache update logic (call update(...))"
