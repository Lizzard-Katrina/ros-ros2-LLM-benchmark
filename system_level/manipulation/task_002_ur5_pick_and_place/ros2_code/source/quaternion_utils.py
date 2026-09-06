"""
Minimal quaternion utility class that replaces pyquaternion.Quaternion
for the subset of functionality used in this package.

This avoids the external pyquaternion dependency.
"""

import numpy as np
import math


class Quaternion:
    """
    A minimal quaternion class compatible with the pyquaternion.Quaternion API
    subset used in this project.

    Internal representation: (w, x, y, z)
    """

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0, axis=None, angle=None, matrix=None):
        if matrix is not None:
            self._from_matrix(matrix)
        elif axis is not None and angle is not None:
            self._from_axis_angle(axis, angle)
        else:
            self.w = float(w)
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
        self._normalize()

    def _from_axis_angle(self, axis, angle):
        axis = np.array(axis, dtype=np.float64)
        n = np.linalg.norm(axis)
        if n < 1e-10:
            self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0
            return
        axis = axis / n
        half = angle / 2.0
        s = math.sin(half)
        self.w = math.cos(half)
        self.x = axis[0] * s
        self.y = axis[1] * s
        self.z = axis[2] * s

    def _from_matrix(self, m):
        """Convert a 3x3 rotation matrix to quaternion."""
        m = np.array(m, dtype=np.float64)
        if m.shape == (4, 4):
            m = m[:3, :3]
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0:
            s = 2.0 * math.sqrt(tr + 1.0)
            self.w = 0.25 * s
            self.x = (m[2, 1] - m[1, 2]) / s
            self.y = (m[0, 2] - m[2, 0]) / s
            self.z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            self.w = (m[2, 1] - m[1, 2]) / s
            self.x = 0.25 * s
            self.y = (m[0, 1] + m[1, 0]) / s
            self.z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            self.w = (m[0, 2] - m[2, 0]) / s
            self.x = (m[0, 1] + m[1, 0]) / s
            self.y = 0.25 * s
            self.z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            self.w = (m[1, 0] - m[0, 1]) / s
            self.x = (m[0, 2] + m[2, 0]) / s
            self.y = (m[1, 2] + m[2, 1]) / s
            self.z = 0.25 * s

    def _normalize(self):
        n = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if n < 1e-10:
            self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0
        else:
            self.w /= n
            self.x /= n
            self.y /= n
            self.z /= n

    @property
    def rotation_matrix(self):
        """Return 3x3 rotation matrix."""
        w, x, y, z = self.w, self.x, self.y, self.z
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
        ], dtype=np.float64)

    @property
    def yaw_pitch_roll(self):
        """Return (yaw, pitch, roll) Euler angles."""
        # yaw (z), pitch (y), roll (x)
        w, x, y, z = self.w, self.x, self.y, self.z
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll

    def rotate(self, vector):
        """Rotate a 3D vector by this quaternion."""
        v = np.array(vector, dtype=np.float64)
        q_vec = np.array([self.x, self.y, self.z])
        t = 2.0 * np.cross(q_vec, v)
        return v + self.w * t + np.cross(q_vec, t)

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
            x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
            y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
            z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
            return Quaternion(w, x, y, z)
        return NotImplemented

    def __repr__(self):
        return f"Quaternion({self.w:.4f}, {self.x:.4f}, {self.y:.4f}, {self.z:.4f})"

    @staticmethod
    def slerp(q1, q2, t):
        """Spherical linear interpolation between two quaternions."""
        dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z

        # If dot is negative, negate one to take shorter path
        if dot < 0:
            q2 = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z)
            dot = -dot

        if dot > 0.9995:
            # Linear interpolation for very close quaternions
            w = q1.w + t * (q2.w - q1.w)
            x = q1.x + t * (q2.x - q1.x)
            y = q1.y + t * (q2.y - q1.y)
            z = q1.z + t * (q2.z - q1.z)
            return Quaternion(w, x, y, z)

        theta_0 = math.acos(dot)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        sin_theta_0 = math.sin(theta_0)

        s1 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s2 = sin_theta / sin_theta_0

        w = s1 * q1.w + s2 * q2.w
        x = s1 * q1.x + s2 * q2.x
        y = s1 * q1.y + s2 * q2.y
        z = s1 * q1.z + s2 * q2.z
        return Quaternion(w, x, y, z)

    @property
    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    @property
    def inverse(self):
        n2 = self.w**2 + self.x**2 + self.y**2 + self.z**2
        c = self.conjugate
        return Quaternion(c.w/n2, c.x/n2, c.y/n2, c.z/n2)