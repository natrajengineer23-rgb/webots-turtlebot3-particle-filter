"""Dependency-light 2-D point-to-point ICP for LiDAR scan matching."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ICPResult:
    increment: tuple
    rmse: float
    correspondences: int
    converged: bool
    iterations: int


def ranges_to_points(ranges, angles, max_range, min_range=0.12, step=2):
    """Convert valid planar ranges to an N x 2 robot-frame point cloud."""
    ranges, angles = np.asarray(ranges, dtype=float), np.asarray(angles, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= min_range) & (ranges < max_range * 0.995)
    ranges, angles = ranges[valid][::step], angles[valid][::step]
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))


def _nearest(source, target):
    """Return nearest target index and squared distance for each source point."""
    squared = np.sum((source[:, None, :] - target[None, :, :]) ** 2, axis=2)
    indices = np.argmin(squared, axis=1)
    return indices, squared[np.arange(len(source)), indices]


def _rigid_transform(source, target):
    source_centre, target_centre = source.mean(axis=0), target.mean(axis=0)
    source_zero, target_zero = source - source_centre, target - target_centre
    u, _, vt = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_centre - rotation @ source_centre
    return rotation, translation


def transform_points(points, matrix):
    return points @ matrix[:2, :2].T + matrix[:2, 2]


def pose_matrix(pose):
    """Return a homogeneous 2-D transform for an (x, y, yaw) pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(((c, -s, x), (s, c, y), (0.0, 0.0, 1.0)), dtype=float)


def icp_2d(source, target, max_iterations=25, tolerance=1e-4,
           max_correspondence=0.35, trim_fraction=0.85, min_pairs=20,
           initial_guess=None):
    """Align source scan to target scan and return target_T_source.

    ``initial_guess`` may be an (x, y, yaw) source pose in the target frame.
    It is useful for scan-to-submap matching where the target is in world
    coordinates and the control command supplies only a search prior.
    """
    source, target = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if len(source) < min_pairs or len(target) < min_pairs:
        return ICPResult((0.0, 0.0, 0.0), float("inf"), 0, False, 0)
    total = np.eye(3) if initial_guess is None else pose_matrix(initial_guess)
    transformed = transform_points(source, total)
    previous_rmse = float("inf")
    pairs = 0
    for iteration in range(1, max_iterations + 1):
        indices, squared = _nearest(transformed, target)
        mask = squared <= max_correspondence ** 2
        if mask.sum() < min_pairs:
            return ICPResult((0.0, 0.0, 0.0), float("inf"), int(mask.sum()), False, iteration)
        accepted_squared = squared[mask]
        trim_limit = np.quantile(accepted_squared, trim_fraction)
        mask &= squared <= trim_limit
        pairs = int(mask.sum())
        if pairs < min_pairs:
            return ICPResult((0.0, 0.0, 0.0), float("inf"), pairs, False, iteration)
        rotation, translation = _rigid_transform(transformed[mask], target[indices[mask]])
        update = np.eye(3)
        update[:2, :2], update[:2, 2] = rotation, translation
        transformed = transform_points(transformed, update)
        total = update @ total
        rmse = float(np.sqrt(np.mean(np.sum((transformed[mask] - target[indices[mask]]) ** 2, axis=1))))
        if abs(previous_rmse - rmse) < tolerance:
            yaw = math.atan2(total[1, 0], total[0, 0])
            return ICPResult((float(total[0, 2]), float(total[1, 2]), yaw), rmse, pairs, True, iteration)
        previous_rmse = rmse
    yaw = math.atan2(total[1, 0], total[0, 0])
    return ICPResult((float(total[0, 2]), float(total[1, 2]), yaw), previous_rmse, pairs, True, max_iterations)
