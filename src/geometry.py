"""Small SE(2) geometry helpers used by mapping and localisation."""
import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose_pose(pose, increment):
    """Compose world_T_body with body_T_next, both represented as (x, y, yaw)."""
    x, y, theta = pose
    dx, dy, dtheta = increment
    c, s = math.cos(theta), math.sin(theta)
    return (
        x + c * dx - s * dy,
        y + s * dx + c * dy,
        wrap_angle(theta + dtheta),
    )


def relative_pose(reference, pose):
    """Express a world-frame pose in the local frame of ``reference``."""
    rx, ry, rtheta = reference
    dx, dy = pose[0] - rx, pose[1] - ry
    c, s = math.cos(rtheta), math.sin(rtheta)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        wrap_angle(pose[2] - rtheta),
    )


def unicycle_increment(linear_velocity: float, angular_velocity: float, dt: float):
    """Exact local-frame increment for a constant unicycle command."""
    dtheta = angular_velocity * dt
    if abs(angular_velocity) < 1e-9:
        return linear_velocity * dt, 0.0, 0.0
    radius = linear_velocity / angular_velocity
    return radius * math.sin(dtheta), radius * (1.0 - math.cos(dtheta)), dtheta


def wheel_commands(linear_velocity, angular_velocity, wheel_radius, axle_length, max_speed):
    """Convert a desired body twist to left/right motor angular commands.

    This is actuation kinematics only; no wheel measurements are read or used.
    """
    left = (linear_velocity - 0.5 * axle_length * angular_velocity) / wheel_radius
    right = (linear_velocity + 0.5 * axle_length * angular_velocity) / wheel_radius
    scale = max(1.0, abs(left) / max_speed, abs(right) / max_speed)
    return left / scale, right / scale
