import re
import pytest
from pathlib import Path

# Locate the source file relative to the test script
CPP_FILE = Path(__file__).resolve().parents[1] / "limo_driver.cpp"

class TestLimoDriverOracle:
    @classmethod
    def setup_class(cls):
        if not CPP_FILE.exists():
            cls.source = ""
            return
        with open(CPP_FILE, 'r') as f:
            cls.source = f.read()

    def get_func_body(self, func_name):
        """Extracts C++ function body using brace matching logic."""
        pattern = rf"{func_name}\s*\([^)]*\)\s*\{{([\s\S]*?)\}}"
        match = re.search(pattern, self.source)
        return match.group(1) if match else ""

    def test_ackermann_inverse_kinematics(self):
        """Verify Ackermann steering geometry derivation."""
        blk = self.get_func_body("twistCmdCallback")
        
        # Check for R = wheelbase / tan(angle) or angle = atan(wheelbase / R)
        has_math = re.search(r"atan\(wheelbase_.*?/.*?r\)", blk) or \
                   re.search(r"wheelbase_.*?/.*?tan", blk)
        
        assert has_math, "Missing Ackermann geometry (atan/tan) using wheelbase."

    def test_steering_limit_clamping(self):
        """Verify mechanical constraint enforcement."""
        blk = self.get_func_body("twistCmdCallback")
        
        has_clamping = "max_inner_angle_" in blk and ("if" in blk or "clamp" in blk)
        assert has_clamping, "Steering angle must be clamped by max_inner_angle_."

    def test_odom_integration_frames(self):
        """Verify global frame projection (Rotation Matrix)."""
        blk = self.get_func_body("publishOdometry")
        
        # Validation of dx = vx*cos - vy*sin and dy = vx*sin + vy*cos
        has_rot_x = re.search(r"position_x_.*?(\+=|=).*?cos\(.*?\).*?\*.*?vx", blk)
        has_rot_y = re.search(r"position_y_.*?(\+=|=).*?sin\(.*?\).*?\*.*?vx", blk)
        
        assert has_rot_x and has_rot_y, "Incorrect velocity projection into global frame."

    def test_mecanum_lateral_awareness(self):
        """Verify holonomic perception for Mecanum mode."""
        blk = self.get_func_body("publishOdometry")
        
        # Ensure vy is integrated into the spatial pose
        is_aware = "lateral_velocity" in blk and "vy" in blk
        assert is_aware, "Lateral velocity (vy) ignored in Mecanum odometry."

    def test_time_differential_consistency(self):
        """Verify Euler integration scaling."""
        blk = self.get_func_body("publishOdometry")
        
        # Ensure displacement uses dt scaling
        assert "dt" in blk and re.search(r"\*.*?dt", blk), "Pose update missing time delta (dt) scaling."

    def test_protocol_bit_shifting(self):
        """Verify low-level packet serialization."""
        blk = self.get_func_body("setMotionCommand")
        
        has_bits = ">> 8" in blk and "& 0x" in blk.lower()
        assert has_bits, "Failed to serialize 16-bit commands into byte-stream."
