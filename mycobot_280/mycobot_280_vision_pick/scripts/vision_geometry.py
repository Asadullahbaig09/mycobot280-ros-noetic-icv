"""Pure geometry for the overhead camera: no ROS imports, so it is unit-testable.

A pixel is back-projected through the pinhole model into a ray in the camera
optical frame, the ray is rotated/translated into the world frame, and
intersected with the horizontal plane z = plane_z (the cubes' top face).
"""
import math

import numpy as np


def pixel_to_plane(u, v, K, R, t, plane_z):
    """World point where pixel (u, v) hits the plane z = plane_z.

    K: 3x3 intrinsics, R/t: rotation and translation of the camera optical
    frame expressed in the world frame (world_p = R @ cam_p + t).
    """
    ray_cam = np.linalg.solve(np.asarray(K, float), [u, v, 1.0])
    ray_world = np.asarray(R, float) @ ray_cam
    if abs(ray_world[2]) < 1e-9:
        raise ValueError("ray is parallel to the plane")
    s = (plane_z - t[2]) / ray_world[2]
    if s <= 0:
        raise ValueError("plane is behind the camera")
    return np.asarray(t, float) + s * ray_world


def square_yaw(p0, p1):
    """Yaw of a square's edge p0->p1 (world xy), folded into [-pi/4, pi/4).

    A square looks identical every 90 degrees, so the smallest equivalent
    rotation is the one the wrist should turn.
    """
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
