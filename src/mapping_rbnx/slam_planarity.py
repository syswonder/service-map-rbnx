# SPDX-License-Identifier: MulanPSL-2.0
"""Reject observations which violate the fixed-plane model of 2D SLAM.

Use the laser's full odom-frame transform at the scan timestamp. Neither
flattening its quaternion nor projecting tilted rays makes their intersections
with furniture/floors equivalent to a horizontal laser scan.
"""
import math


class PlanarScanGuard:
    def __init__(self, max_tilt_deg=3., max_height_change_m=.04):
        if not (math.isfinite(max_tilt_deg) and 0 < max_tilt_deg < 90
                and math.isfinite(max_height_change_m) and max_height_change_m > 0):
            raise ValueError("planar scan limits must be finite and positive")
        self.max_tilt = math.radians(max_tilt_deg)
        self.max_height_change = max_height_change_m
        self.reference_height = None

    def check(self, height, quaternion):
        values = (height, *quaternion)
        if not all(math.isfinite(v) for v in values):
            return "nonfinite laser transform"
        norm = math.sqrt(sum(v*v for v in quaternion))
        if norm < 1e-9:
            return "invalid laser orientation"
        x, y, z, w = (v/norm for v in quaternion)
        # Parallel and upside-down mounting both describe a horizontal plane.
        tilt = math.acos(min(1., abs(1.-2.*(x*x+y*y))))
        if tilt > self.max_tilt:
            return f"laser plane tilted {math.degrees(tilt):.1f} deg"
        if self.reference_height is None:
            self.reference_height = height
        if abs(height-self.reference_height) > self.max_height_change:
            return f"laser height changed {height-self.reference_height:.3f} m"
        return None
