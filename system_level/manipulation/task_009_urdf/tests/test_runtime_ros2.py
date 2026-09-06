"""
Runtime test for task_009_urdf.

Validates the URDF, SRDF, and joint_limits.yaml files by parsing them
and checking structural/semantic correctness at runtime.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
import pytest


# Locate files relative to this test file (package root)
PKG_ROOT = Path(__file__).resolve().parent
URDF_FILE = PKG_ROOT / "arm_urdf.urdf"
SRDF_FILE = PKG_ROOT / "manipulator.srdf"
LIMITS_FILE = PKG_ROOT / "joint_limits.yaml"


def _parse_xml(path):
    """Parse an XML file and return the ElementTree root."""
    tree = ET.parse(str(path))
    return tree.getroot()


# ── URDF Tests ──────────────────────────────────────────────────────────────

class TestURDFKinematics:
    """Validate the kinematic chain in the URDF."""

    @pytest.fixture(autouse=True)
    def load_urdf(self):
        self.root = _parse_xml(URDF_FILE)

    def _find_joint(self, name):
        for j in self.root.findall('joint'):
            if j.get('name') == name:
                return j
        raise AssertionError(f"Joint '{name}' not found in URDF")

    def _find_link(self, name):
        for l in self.root.findall('link'):
            if l.get('name') == name:
                return l
        raise AssertionError(f"Link '{name}' not found in URDF")

    def test_joint2_axis_is_pitch(self):
        """Joint2 must rotate around the Y-axis (pitch)."""
        j2 = self._find_joint('joint2')
        axis = j2.find('axis')
        assert axis is not None, "joint2 missing <axis>"
        assert axis.get('xyz') == '0 1 0', \
            f"joint2 axis should be '0 1 0', got '{axis.get('xyz')}'"

    def test_link3_visual_origin(self):
        """Link3 visual origin z must be 0.15 (half of 0.3m cylinder)."""
        link3 = self._find_link('link3')
        vis = link3.find('visual')
        assert vis is not None
        origin = vis.find('origin')
        assert origin is not None
        xyz = origin.get('xyz')
        parts = xyz.split()
        assert len(parts) == 3
        assert float(parts[2]) == pytest.approx(0.15, abs=1e-6), \
            f"Link3 visual origin z should be 0.15, got {parts[2]}"

    def test_joint3_origin(self):
        """Joint3 origin z must be 0.28 (0.3 - 0.02 offset)."""
        j3 = self._find_joint('joint3')
        origin = j3.find('origin')
        assert origin is not None
        xyz = origin.get('xyz')
        parts = xyz.split()
        assert float(parts[2]) == pytest.approx(0.28, abs=1e-6), \
            f"Joint3 origin z should be 0.28, got {parts[2]}"

    def test_kinematic_chain_completeness(self):
        """All 6 arm joints + 2 finger joints must exist."""
        joint_names = {j.get('name') for j in self.root.findall('joint')}
        expected = {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
                    'f_joint1', 'f_joint2'}
        assert expected.issubset(joint_names), \
            f"Missing joints: {expected - joint_names}"

    def test_all_links_present(self):
        """All links from base_link through link6 plus finger links must exist."""
        link_names = {l.get('name') for l in self.root.findall('link')}
        expected = {'base_link', 'link1', 'link2', 'link3', 'link4', 'link5',
                    'link6', 'f_link1', 'f_link2'}
        assert expected.issubset(link_names), \
            f"Missing links: {expected - link_names}"


# ── SRDF Tests ──────────────────────────────────────────────────────────────

class TestSRDFSemantics:
    """Validate the SRDF planning group and collision matrix."""

    @pytest.fixture(autouse=True)
    def load_srdf(self):
        self.root = _parse_xml(SRDF_FILE)

    def test_arm_group_chain(self):
        """The 'arm' group must be a chain from base_link to link6."""
        for group in self.root.findall('group'):
            if group.get('name') == 'arm':
                chain = group.find('chain')
                assert chain is not None, "'arm' group has no <chain>"
                assert chain.get('base_link') == 'base_link'
                assert chain.get('tip_link') == 'link6'
                return
        pytest.fail("No group named 'arm' found in SRDF")

    def test_gripper_group_exists(self):
        """A 'gripper' group must exist."""
        names = [g.get('name') for g in self.root.findall('group')]
        assert 'gripper' in names

    def test_acm_link5_link6(self):
        """Adjacent collision between link5 and link6 must be disabled."""
        for dc in self.root.findall('disable_collisions'):
            l1, l2 = dc.get('link1'), dc.get('link2')
            if {l1, l2} == {'link5', 'link6'}:
                assert dc.get('reason') == 'Adjacent'
                return
        pytest.fail("disable_collisions for link5-link6 not found")

    def test_acm_has_multiple_entries(self):
        """ACM should have at least 7 adjacent pairs."""
        dc_list = self.root.findall('disable_collisions')
        adjacent = [dc for dc in dc_list if dc.get('reason') == 'Adjacent']
        assert len(adjacent) >= 7, \
            f"Expected >=7 Adjacent disable_collisions, got {len(adjacent)}"


# ── Joint Limits YAML Tests ────────────────────────────────────────────────

class TestJointLimits:
    """Validate the joint_limits.yaml structure and values."""

    @pytest.fixture(autouse=True)
    def load_yaml(self):
        with open(LIMITS_FILE, 'r') as f:
            data = yaml.safe_load(f)
        self.limits = data['joint_limits']

    def test_joint1_velocity_flag(self):
        """joint1 must have has_velocity_limits: true."""
        assert self.limits['joint1']['has_velocity_limits'] is True

    def test_joint1_acceleration_flag(self):
        """joint1 must have has_acceleration_limits: true."""
        assert self.limits['joint1']['has_acceleration_limits'] is True

    def test_f_joint1_acceleration(self):
        """f_joint1 must have acceleration limits enabled and set to 0.5."""
        fj1 = self.limits['f_joint1']
        assert fj1['has_acceleration_limits'] is True
        assert fj1['max_acceleration'] == pytest.approx(0.5)

    def test_f_joint2_acceleration(self):
        """f_joint2 must have acceleration limits enabled and set to 0.5."""
        fj2 = self.limits['f_joint2']
        assert fj2['has_acceleration_limits'] is True
        assert fj2['max_acceleration'] == pytest.approx(0.5)

    def test_all_joints_have_velocity_limits(self):
        """Every joint must have has_velocity_limits: true."""
        for name, cfg in self.limits.items():
            assert cfg.get('has_velocity_limits') is True, \
                f"{name} missing has_velocity_limits: true"

    def test_all_joints_have_position_limits(self):
        """Every joint must have has_position_limits: true."""
        for name, cfg in self.limits.items():
            assert cfg.get('has_position_limits') is True, \
                f"{name} missing has_position_limits: true"

    def test_all_arm_joints_present(self):
        """All 6 arm joints and 2 finger joints must be in the YAML."""
        expected = {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
                    'f_joint1', 'f_joint2'}
        assert expected.issubset(set(self.limits.keys()))


# ── Regex-based checks (mirrors the static oracle) ─────────────────────────

class TestStaticOracleCompat:
    """Re-run the exact regex patterns from the static oracle to ensure compat."""

    def test_urdf_j2_axis(self):
        content = URDF_FILE.read_text()
        pat = r'<joint name="joint2"[^>]*>.*?<axis xyz="0 1 0"'
        assert re.search(pat, content, re.DOTALL)

    def test_urdf_l3_origin(self):
        content = URDF_FILE.read_text()
        pat = r'<link name="link3"[^>]*>.*?<origin[^>]*xyz="0 0 0\.15"'
        assert re.search(pat, content, re.DOTALL)

    def test_urdf_j3_origin(self):
        content = URDF_FILE.read_text()
        pat = r'<joint name="joint3"[^>]*>.*?<origin[^>]*xyz="0 0 0\.28"'
        assert re.search(pat, content, re.DOTALL)

    def test_srdf_arm_chain(self):
        content = SRDF_FILE.read_text()
        pat = r'<group name="arm">.*?<chain base_link="base_link" tip_link="link6"'
        assert re.search(pat, content, re.DOTALL)

    def test_srdf_acm(self):
        content = SRDF_FILE.read_text()
        pat = r'<disable_collisions link1="link5" link2="link6" reason="Adjacent"'
        assert re.search(pat, content)

    def test_yaml_vel_flag(self):
        content = LIMITS_FILE.read_text()
        pat = r'joint1:.*?has_velocity_limits:\s*true'
        assert re.search(pat, content, re.DOTALL)

    def test_yaml_f_joint1_acc(self):
        content = LIMITS_FILE.read_text()
        pat = r'f_joint1:.*?has_acceleration_limits:\s*true.*?max_acceleration:\s*0\.5'
        assert re.search(pat, content, re.DOTALL)