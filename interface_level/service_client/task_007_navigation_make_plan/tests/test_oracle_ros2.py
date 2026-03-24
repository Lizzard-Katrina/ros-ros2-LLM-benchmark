import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "amcl_node.cpp"


def _code():
    return CPP_FILE.read_text(encoding="utf-8", errors="ignore")


def _assert_has(pat: str, code: str, msg: str, flags=0):
    if not re.search(pat, code, flags):
        raise AssertionError(msg + f"\nMissing pattern: {pat}\nFile: {CPP_FILE}")


def _assert_not_has(pat: str, code: str, msg: str, flags=0):
    if re.search(pat, code, flags):
        raise AssertionError(msg + f"\nForbidden pattern: {pat}\nFile: {CPP_FILE}")


def _extract_requestmap_body(code: str):
    m = re.search(
        r"requestMap\s*\([^)]*\)\s*\{([\s\S]*?)\n\}",
        code,
        re.DOTALL,
    )
    return m.group(1) if m else code


class TestOracleNavigationMakePlan_ServiceClient_Effective:

    # 1️⃣ ROS2 client existence（基础接口迁移）
    def test_01_ros2_service_client_used(self):
        code = _code()
        _assert_has(
            r"create_client\s*<\s*nav_msgs::srv::GetMap\s*>",
            code,
            "Must use ROS2 create_client<nav_msgs::srv::GetMap>.",
        )

    # 2️⃣ wait_for_service retry 语义（工作流语义）
    def test_02_wait_for_service_retry(self):
        blk = _extract_requestmap_body(_code())
        _assert_has(
            r"wait_for_service\s*\(",
            blk,
            "Must wait for service availability.",
        )

    # 3️⃣ ⭐ 核心语义：锁语义迁移（高区分度）
    def test_03_requestmap_has_mutex_lock_guard(self):
        blk = _extract_requestmap_body(_code())

        _assert_has(
            r"(scoped_lock|lock_guard|unique_lock)[\s\S]{0,120}\bconfiguration_mutex_\b",
            blk,
            "Must preserve mutex locking semantics from ROS1 (configuration_mutex_).",
            flags=re.DOTALL,
        )

    # 4️⃣ response → handleMapMessage 语义链（防假实现）
    def test_04_response_map_flow(self):
        blk = _extract_requestmap_body(_code())

        _assert_has(
            r"handleMapMessage\s*\([\s\S]{0,80}(->|\.)\s*map",
            blk,
            "Map from service response must flow into handleMapMessage(...).",
            flags=re.DOTALL,
        )
