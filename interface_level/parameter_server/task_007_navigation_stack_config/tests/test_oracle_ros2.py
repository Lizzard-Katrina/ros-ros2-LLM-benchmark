import re
import pytest
from pathlib import Path

# 假设生成的代码路径
CPP_FILE = Path(__file__).resolve().parents[1] / "amcl_node.cpp"

@pytest.fixture
def code_content():
    """读取模型生成的 C++ 代码内容"""
    if not CPP_FILE.exists():
        pytest.fail(f"Source file {CPP_FILE} not found.")
    return CPP_FILE.read_text(encoding="utf-8")

def test_parameter_declaration_logic(code_content):
    patterns = [
        r'declare_parameter<\s*int\s*>\(\s*["\']min_particles["\']\s*,\s*100\s*\)',
        r'declare_parameter<\s*int\s*>\(\s*["\']max_particles["\']\s*,\s*5000\s*\)',
        r'declare_parameter<\s*std::string\s*>\(\s*["\']odom_frame_id["\']\s*,\s*["\']odom["\']\s*\)'
    ]
    for p in patterns:
        assert re.search(p, code_content), f"语义缺失：未正确声明参数或默认值不符合要求 (Pattern: {p})"

def test_callback_binding_logic(code_content):
    pattern = r'add_on_set_parameters_callback\(\s*(?:std::bind|\[|this)'
    assert re.search(pattern, code_content), "语义失败：未注册 ROS 2 动态参数回调 (add_on_set_parameters_callback)"

def test_min_max_constraint_logic(code_content):
    logic_pattern = r'min_particles\s*[><=]+\s*max_particles'
    failure_pattern = r'\.successful\s*=\s*false'
    
    assert re.search(logic_pattern, code_content), "逻辑缺失：回调函数中未对比 min/max particles 的合法性"
    assert re.search(failure_pattern, code_content), "语义缺失：当参数校验失败时，未设置 SetParametersResult 为 false"

def test_parameter_type_safety(code_content):
    assert ".as_int()" in code_content, "语义错误：未正确使用 .as_int() 提取整数参数"
    assert ".as_string()" in code_content or ".as_double()" in code_content, "语义错误：未正确提取字符串或浮点参数"

def test_internal_state_sync(code_content):
    pattern = r'min_particles_\s*=\s*\w+\.as_int\(\)'
    assert re.search(pattern, code_content), "语义失败：参数校验成功后未同步更新内部成员变量 (min_particles_)"

def test_no_legacy_artifacts(code_content):
    forbidden = [
        r'private_nh_',
        r'ros::NodeHandle',
        r'dynamic_reconfigure',
        r'AMCLConfig',
        r'\.getParam'
    ]
    for f in forbidden:
        assert not re.search(f, code_content), f"迁移不彻底：代码中检测到 ROS 1 遗留符号 '{f}'"

def test_callback_return_type(code_content):
    assert "SetParametersResult" in code_content, "语义失败：回调函数未返回 SetParametersResult 类型"
    assert ".reason" in code_content, "语义缺失：拒绝更新时未通过 .reason 字段提供反馈信息"
