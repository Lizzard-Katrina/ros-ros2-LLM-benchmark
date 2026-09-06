"""Runs a candidate ROS2 package through a real Docker build + runtime test.

This is Tier1 (colcon build) + Tier3 (actual node execution) from the
benchmark design discussion. Tier2 (the existing static oracle regex tests)
is checked separately in Python, no Docker needed for that.
"""

import os
import re
import shutil
import subprocess
import tempfile
import uuid

_MISSING_PKG_RE = re.compile(
    r'package configuration file provided by "([\w-]+)"'
)


def _find_missing_ros_packages(build_log: str) -> list:
    return sorted(set(_MISSING_PKG_RE.findall(build_log)))

BASE_IMAGE = os.environ.get("GROUNDTRUTH_BASE_IMAGE", "osrf/ros:humble-desktop")
SETUP_CMD = (
    "apt-get update -qq && "
    "apt-get install -y -qq python3-colcon-common-extensions python3-pip "
    "ros-humble-rosidl-default-generators >/dev/null 2>&1 && "
    "pip3 install -q pytest"
)


class _FakeCompleted:
    """Stand-in for subprocess.CompletedProcess when a command times out, so
    callers never have to special-case a raised TimeoutExpired."""

    def __init__(self, stdout, stderr, returncode=124):
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode


def _run(cmd, timeout=None):
    try:
        return subprocess.run(
            cmd, capture_output=True, timeout=timeout,
            encoding="utf-8", errors="replace",
        )
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or "") if isinstance(e.stdout, str) else (e.stdout or b"").decode("utf-8", "replace")
        err = (e.stderr or "") if isinstance(e.stderr, str) else (e.stderr or b"").decode("utf-8", "replace")
        return _FakeCompleted(
            out, err + f"\n[groundtruth_pipeline] command timed out after {timeout}s: {cmd}",
        )


class DockerVerifyResult:
    def __init__(self):
        self.build_ok = False
        self.build_log = ""
        self.test_ok = False
        self.test_log = ""
        self.stage_failed = None  # "build" | "test" | None


def verify_package(package_dir: str, package_name: str, timeout_build: int = 900,
                    timeout_test: int = 200) -> DockerVerifyResult:
    """package_dir must contain package.xml + (setup.py/setup.cfg or CMakeLists.txt),
    the node source, and test_runtime_ros2.py at its root."""
    result = DockerVerifyResult()
    container = f"gt_{package_name}_{uuid.uuid4().hex[:8]}"

    _run(["docker", "rm", "-f", container])
    run_res = _run(["docker", "run", "-d", "--name", container, BASE_IMAGE, "tail", "-f", "/dev/null"])
    if run_res.returncode != 0:
        result.build_log = f"docker run failed: {run_res.stderr}"
        result.stage_failed = "build"
        return result

    try:
        _run(["docker", "exec", container, "bash", "-c", SETUP_CMD], timeout=300)
        _run(["docker", "exec", container, "mkdir", "-p", f"/ros2_ws/src/{package_name}"])
        cp_res = _run(["docker", "cp", f"{package_dir}/.", f"{container}:/ros2_ws/src/{package_name}/"])
        if cp_res.returncode != 0:
            result.build_log = f"docker cp failed: {cp_res.stderr}"
            result.stage_failed = "build"
            return result

        build_cmd = (
            "source /opt/ros/humble/setup.bash && cd /ros2_ws && "
            "colcon build --event-handlers console_direct+ 2>&1"
        )
        build_res = _run(["docker", "exec", container, "bash", "-c", build_cmd], timeout=timeout_build)
        result.build_log = build_res.stdout + build_res.stderr

        if build_res.returncode != 0:
            missing_pkgs = _find_missing_ros_packages(result.build_log)
            if missing_pkgs:
                apt_pkgs = " ".join(f"ros-humble-{p.replace('_', '-')}" for p in missing_pkgs)
                heal_cmd = f"apt-get install -y -qq {apt_pkgs} >/dev/null 2>&1"
                heal_res = _run(["docker", "exec", container, "bash", "-c", heal_cmd], timeout=180)
                if heal_res.returncode == 0:
                    build_res = _run(["docker", "exec", container, "bash", "-c", build_cmd], timeout=timeout_build)
                    result.build_log += (
                        f"\n[groundtruth_pipeline] auto-installed missing dependency package(s) "
                        f"{apt_pkgs} and retried build:\n" + build_res.stdout + build_res.stderr
                    )

        if build_res.returncode != 0:
            result.stage_failed = "build"
            return result
        result.build_ok = True

        # Don't assume the LLM put it exactly at package root -- search for it anywhere
        # under the package (it may have followed ROS2 convention and used test/).
        find_res = _run([
            "docker", "exec", container, "bash", "-c",
            f"find /ros2_ws/src/{package_name} -name test_runtime_ros2.py | head -1",
        ])
        test_path = (find_res.stdout or "").strip()
        if not test_path:
            result.test_log = "test_runtime_ros2.py not found anywhere under the package"
            result.stage_failed = "test"
            return result

        # Use the `pytest` entry-point script directly, NOT `python3 -m pytest` --
        # `-m` prepends the current working directory to sys.path, which can shadow
        # a real installed package (e.g. ROS2's own `launch`) if the task's package
        # happens to contain a same-named directory (e.g. a conventional launch/ folder).
        test_cmd = (
            "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && "
            f"cd /ros2_ws/src/{package_name} && "
            f"timeout 170 pytest-3 {test_path} -q --tb=short 2>&1"
        )
        test_res = _run(["docker", "exec", container, "bash", "-c", test_cmd], timeout=timeout_test)
        result.test_log = test_res.stdout + test_res.stderr
        result.test_ok = test_res.returncode == 0
        if not result.test_ok:
            result.stage_failed = "test"
        return result
    finally:
        _run(["docker", "rm", "-f", container])


def materialize_package(files: dict, package_name: str) -> str:
    """Writes an LLM-produced {relative_path: content} file set to a temp package dir."""
    tmp_dir = tempfile.mkdtemp(prefix=f"gt_pkg_{package_name}_")
    for rel_path, content in files.items():
        full_path = os.path.join(tmp_dir, rel_path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        with open(full_path, "w", encoding="utf-8") as f:
            f.write(content)
    return tmp_dir


def cleanup_package(tmp_dir: str):
    shutil.rmtree(tmp_dir, ignore_errors=True)
