"""Pixel + depth → 3D point backprojection (no ROS2 dependency).

Uses the pinhole camera model to project a 2D pixel coordinate with a
known depth value into a 3D point in the camera frame.  Assumes the
image has already been rectified (no distortion).
"""

import numpy as np


def scale_depth(
    d_relative: float,
    a: float,
    b: float,
    model_type: str = 'inverse',
) -> float:
    """Convert relative depth from DA V2 to metric depth (meters).

    Parameters
    ----------
    d_relative : float
        Relative depth value from DA V2 (0.0–1.0).
    a, b : float
        Calibration parameters from ``depth_calibration.yaml``.
    model_type : str
        ``"inverse"`` for ``Z = a / d + b``,
        ``"inverse_no_offset"`` for ``Z = a / d``,
        ``"linear"`` for ``Z = a * d + b``.
    """
    if model_type in ('inverse', 'inverse_no_offset'):
        return a / max(d_relative, 1e-6) + b
    else:  # linear
        return a * d_relative + b


def backproject(
    u: float,
    v: float,
    depth: float,
    K: np.ndarray,
) -> np.ndarray:
    """Back-project a pixel into the camera coordinate frame.

    Parameters
    ----------
    u, v : float
        Pixel coordinates (column, row) in the rectified image.
    depth : float
        Depth value at (u, v).  In Phase 3 this is the relative depth
        from DA V2 (0.0–1.0); metric scaling comes in Phase 4.
    K : np.ndarray
        3×3 camera intrinsic matrix (after rectification = pinhole).

    Returns
    -------
    np.ndarray
        3D point ``[X, Y, Z]`` in the camera coordinate frame.
    """
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    Z = depth
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    return np.array([X, Y, Z], dtype=np.float64)
