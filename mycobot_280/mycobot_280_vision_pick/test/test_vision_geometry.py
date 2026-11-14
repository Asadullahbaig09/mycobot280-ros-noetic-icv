#!/usr/bin/env python3
"""Self-check for vision_geometry: python3 test/test_vision_geometry.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from vision_geometry import pixel_to_plane, square_yaw  # noqa: E402

# Camera 0.65 m above (0.08, 0), looking straight down. Optical frame in world:
# optical z -> world -z, optical x (image right) -> world -y, optical y (image down) -> world -x
R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
t = np.array([0.08, 0.0, 0.65])
K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], float)

# Principal point lands directly below the camera
assert np.allclose(pixel_to_plane(320, 240, K, R, t, 0.0), [0.08, 0.0, 0.0])

# Round trip: project a world point into the image, back-project it again
p = np.array([0.17, 0.07, 0.025])
cam = R.T @ (p - t)
u, v = (K @ (cam / cam[2]))[:2]
assert np.allclose(pixel_to_plane(u, v, K, R, t, 0.025), p)

# Plane above the camera is rejected
try:
    pixel_to_plane(320, 240, K, R, t, 1.0)
    raise AssertionError("expected ValueError")
except ValueError:
    pass

# Yaw folds into [-45, 45) degrees: a square rotated by 100 deg looks like 10 deg
assert math.isclose(square_yaw((0, 0), (1, 0)), 0.0, abs_tol=1e-9)
a = math.radians(100)
assert math.isclose(square_yaw((0, 0), (math.cos(a), math.sin(a))), math.radians(10), abs_tol=1e-9)
a = math.radians(-50)
assert math.isclose(square_yaw((0, 0), (math.cos(a), math.sin(a))), math.radians(40), abs_tol=1e-9)

print("vision_geometry: all checks passed")
