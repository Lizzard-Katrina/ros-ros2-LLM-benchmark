#!/usr/bin/env python3
"""
Quaternion helper functions replacing transforms3d dependency.
All quaternions use (w, x, y, z) internal convention unless noted.
"""
import math


def euler2quat(ai, aj, ak):
    """
    Convert Euler angles (roll=ai, pitch=aj, yaw=ak) to quaternion.
    Uses 'sxyz' convention (static axes, x-y-z).
    Returns (w, x, y, z).
    """
    ai_half = ai / 2.0
    aj_half = aj / 2.0
    ak_half = ak / 2.0

    ci = math.cos(ai_half)
    si = math.sin(ai_half)
    cj = math.cos(aj_half)
    sj = math.sin(aj_half)
    ck = math.cos(ak_half)
    sk = math.sin(ak_half)

    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    w = cj * cc + sj * ss
    x = cj * sc - sj * cs
    y = cj * ss + sj * cc
    z = cj * cs - sj * sc

    return (w, x, y, z)


def qmult(q1, q2):
    """
    Multiply two quaternions q1 * q2.
    Each quaternion is (w, x, y, z).
    Returns (w, x, y, z).
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (w, x, y, z)