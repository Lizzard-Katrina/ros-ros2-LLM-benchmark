import re
import pytest
from pathlib import Path

LOCATE_PY_FILE = Path(__file__).resolve().parents[1] / "detection_2d_to_3d.py"

class TestStretchPerceptionOracle:
    @classmethod
    def setup_class(cls):
        if not LOCATE_PY_FILE.exists():
            cls.source = ""
            return
        with open(LOCATE_PY_FILE, 'r') as f:
            cls.source = f.read()

    def test_intrinsic_decomposition(self):
        """Verify if model deconstructs CameraInfo K matrix (fx, cx, fy, cy)."""
        assert "camera_matrix[0,0]" in self.source or "camera_matrix[0][0]" in self.source, "Missing fx extraction."
        assert "camera_matrix[0,2]" in self.source or "camera_matrix[0][2]" in self.source, "Missing cx extraction."

    def test_depth_unit_scaling(self):
        """Verify mm to meters conversion (z / 1000.0)."""
        has_scaling = "/ 1000" in self.source or "* 0.001" in self.source or "/1000" in self.source
        assert has_scaling, "Depth must be scaled from mm to meters."

    def test_pinhole_projection_logic(self):
        """Verify the inverse pinhole formula: (x - cx) / fx * z."""
        match_x = re.search(r"x\s*-\s*c_x.*?\/\s*f_x", self.source)
        match_y = re.search(r"y\s*-\s*c_y.*?\/\s*f_y", self.source)
        assert match_x and match_y, "Incorrect pinhole projection implementation."

    def test_ray_plane_analytical_solution(self):
        """Verify intersection formula: (d / dot(n, ray)) * ray."""
        has_dot = "np.matmul" in self.source or "@" in self.source or "np.dot" in self.source
        has_plane_math = "d /" in self.source or "d/" in self.source
        assert has_dot and has_plane_math, "Missing ray-plane intersection analytical math."

    def test_vector_flattening(self):
        """Verify engineering detail: flattening the resulting 3D point."""
        assert ".flatten()" in self.source, "Final 3D point must be flattened to a 1D array."

    def test_median_noise_reduction(self):
        """Verify use of median for robust depth in bounding boxes."""
        assert "np.median" in self.source, "Must use np.median for robust box depth estimation."
