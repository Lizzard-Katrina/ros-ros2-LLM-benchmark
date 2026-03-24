import re
from pathlib import Path

CLIENT_FILE = Path(__file__).resolve().parents[1] / "gazebo_interface.py"


def read_file(path):
    return path.read_text()



def test_return_depends_on_service_response():
    """Return value must depend on service response (not constant True/False)"""
    code = read_file(CLIENT_FILE)
    # 必须出现 .success
    assert re.search(r"\.success", code), "Return value must depend on service response success field"

    # 不允许 return True / False 单行常量
    assert not re.search(r"return\s+(True|False)\s*$", code, re.MULTILINE), \
        "Return value must not be a hardcoded literal"

def test_failure_path_exists():
    """There must be at least one path where failure leads to False"""
    code = read_file(CLIENT_FILE)
    # 可以是 except / future result check / None check
    assert re.search(r"(except\s+.*:|if\s+.*\.done\(\)|if\s+.*is\s+None)", code), \
        "There must be a path where service failure or exception maps to False"
def test_return_after_waiting_service():
    """Return must happen after waiting for the service or future completion"""
    code = read_file(CLIENT_FILE)
    # ROS2 future pattern: spin_until_future_complete 或 result()
    assert re.search(r"(spin_until_future_complete|\.result\(\))", code), \
        "Return must occur after waiting for service/future completion"

def test_response_used_in_return():
    """Return value must actually use the response object"""
    code = read_file(CLIENT_FILE)
    assert re.search(r"return\s+.*response\.success", code) or \
           re.search(r"success\s*=\s*.*response\.success", code), \
        "Return value must actually use the response object"

def test_no_trivial_success_assignment():
    """Success variable cannot be a hardcoded True/False"""
    code = read_file(CLIENT_FILE)
    assert not re.search(r"success\s*=\s*(True|False)", code), \
        "Success must depend on response/future, not a constant"

def test_exception_handled():
    """Must handle exceptions, e.g., try/except"""
    code = read_file(CLIENT_FILE)
    assert re.search(r"try\s*:|except\s+.*:", code), \
        "Service call exceptions must be handled"

def test_multiple_service_calls_use_response():
    """If multiple service calls exist, each call's response must be used"""
    code = read_file(CLIENT_FILE)
    # 查找 spawn_sdf / spawn_urdf / set_model_config 的 response usage
    for fn in ["spawn_sdf_model_client", "spawn_urdf_model_client", "set_model_configuration_client"]:
        assert re.search(rf"{fn}.*response", code, re.DOTALL), \
            f"{fn} must use the response object in return/logic"

