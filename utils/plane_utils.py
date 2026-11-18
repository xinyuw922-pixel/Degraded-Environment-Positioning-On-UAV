"""Independent numerical helpers for plane estimation and frame levelling."""

import numpy as np


def estimate_near_horizontal_plane(points, threshold=0.012, trials=300, seed=7):
    """Fit a robust plane n·p+d=0 with normal within 20 degrees of +Z."""
    if len(points) < 500 or not np.isfinite(points).all():
        raise ValueError('Need at least 500 finite candidate floor points')
    random = np.random.default_rng(seed)
    best = None
    for attempt in range(trials):
        sample = points[random.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1]-sample[0], sample[2]-sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < np.cos(np.radians(20)):
            continue
        offset = -float(normal @ sample[0])
        mask = np.abs(points @ normal + offset) < threshold
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < 500:
        raise ValueError('No sufficiently supported floor-plane candidate')
    selected = points[best]
    center = selected.mean(axis=0)
    _, singular, vectors = np.linalg.svd(selected-center, full_matrices=False)
    if singular[1] / np.sqrt(len(selected)) < 0.2:
        raise ValueError('Floor support has insufficient extent in two directions')
    normal = vectors[-1]
    if normal[2] < 0:
        normal = -normal
    offset = -float(normal @ center)
    mask = np.abs(points @ normal + offset) < threshold
    return normal, offset, mask


def quaternion_matrix(quaternion):
    """Convert one finite, nonzero wxyz quaternion to an active rotation matrix."""
    quaternion = np.asarray(quaternion, dtype=float)
    if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-8:
        raise ValueError('Invalid quaternion')
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def plane_level_transform(normal, offset):
    """Return R,t,wxyz mapping an upward-facing plane onto world Z=0."""
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal)
    if normal[2] <= 0:
        raise ValueError('Expected upward-facing floor normal')
    cross = np.cross(normal, [0, 0, 1])
    quaternion = np.concatenate(([1+normal[2]], cross))
    quaternion /= np.linalg.norm(quaternion)
    return quaternion_matrix(quaternion), np.array([0, 0, offset]), quaternion
