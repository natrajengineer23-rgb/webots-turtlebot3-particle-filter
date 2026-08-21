import math
from collections import deque
import numpy as np
from .geometry import wrap_angle


def distance_field(occupied):
    """Eight-connected distance-to-nearest-obstacle field in grid-cell units."""
    h, w = occupied.shape
    dist = np.full((h, w), np.inf, dtype=np.float32)
    queue = deque()
    for y, x in np.argwhere(occupied):
        dist[y, x] = 0.0
        queue.append((x, y))
    neighbours = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                  (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
    while queue:
        x, y = queue.popleft()
        for dx, dy, cost in neighbours:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] > dist[y, x] + cost:
                dist[ny, nx] = dist[y, x] + cost
                queue.append((nx, ny))
    return dist


class ParticleFilter:
    def __init__(self, grid, count=500, sensor_sigma=0.18, random_probability=0.05,
                 seed=7, measurement_power=1.0):
        self.grid, self.count = grid, int(count)
        self.sigma, self.random_probability = sensor_sigma, random_probability
        self.measurement_power = float(measurement_power)
        self.rng = np.random.default_rng(seed)
        self.free_cells = np.argwhere(grid.probability() < 0.35)
        if not len(self.free_cells):
            raise ValueError("Map has no free cells")
        self.particles = self._sample_free(self.count)
        self.weights = np.full(self.count, 1.0 / self.count)
        self.distance = distance_field(grid.probability() > 0.65) * grid.resolution

    def _sample_free(self, count):
        chosen = self.free_cells[self.rng.integers(0, len(self.free_cells), count)]
        return np.column_stack((
            self.grid.origin[0] + (chosen[:, 1] + 0.5) * self.grid.resolution,
            self.grid.origin[1] + (chosen[:, 0] + 0.5) * self.grid.resolution,
            self.rng.uniform(-math.pi, math.pi, count)))

    def predict_command(self, linear_velocity, angular_velocity, dt, noise):
        """Velocity motion model driven by commands, never encoder measurements.

        `noise` is [linear velocity std (m/s), angular velocity std (rad/s)].
        """
        if dt <= 0.0:
            return
        linear = self.rng.normal(linear_velocity, noise[0], self.count)
        angular = self.rng.normal(angular_velocity, noise[1], self.count)
        heading = self.particles[:, 2]
        dtheta = angular * dt
        straight = np.abs(angular) < 1e-7
        curved = ~straight
        self.particles[straight, 0] += linear[straight] * dt * np.cos(heading[straight])
        self.particles[straight, 1] += linear[straight] * dt * np.sin(heading[straight])
        radius = np.zeros(self.count)
        radius[curved] = linear[curved] / angular[curved]
        self.particles[curved, 0] += radius[curved] * (
            np.sin(heading[curved] + dtheta[curved]) - np.sin(heading[curved])
        )
        self.particles[curved, 1] += radius[curved] * (
            -np.cos(heading[curved] + dtheta[curved]) + np.cos(heading[curved])
        )
        self.particles[:, 2] = (heading + dtheta + math.pi) % (2 * math.pi) - math.pi

    def update(self, ranges, angles, max_range, beam_step=8):
        valid = np.isfinite(ranges) & (ranges > 0.02) & (ranges < max_range * 0.995)
        ranges, angles = np.asarray(ranges)[valid][::beam_step], np.asarray(angles)[valid][::beam_step]
        if not len(ranges):
            return
        log_weights = np.log(self.weights + 1e-300)
        measurement_log = np.zeros(self.count)
        particle_gx = np.floor((self.particles[:, 0] - self.grid.origin[0]) / self.grid.resolution).astype(int)
        particle_gy = np.floor((self.particles[:, 1] - self.grid.origin[1]) / self.grid.resolution).astype(int)
        pose_inside = ((particle_gx >= 0) & (particle_gx < self.grid.width) &
                       (particle_gy >= 0) & (particle_gy < self.grid.height))
        valid_pose = np.zeros(self.count, dtype=bool)
        valid_pose[pose_inside] = self.grid.probability()[particle_gy[pose_inside], particle_gx[pose_inside]] < 0.35
        log_weights[~valid_pose] -= 50.0
        for distance, bearing in zip(ranges, angles):
            beam = self.particles[:, 2] + bearing
            ex = self.particles[:, 0] + distance * np.cos(beam)
            ey = self.particles[:, 1] + distance * np.sin(beam)
            gx = np.floor((ex - self.grid.origin[0]) / self.grid.resolution).astype(int)
            gy = np.floor((ey - self.grid.origin[1]) / self.grid.resolution).astype(int)
            inside = (gx >= 0) & (gx < self.grid.width) & (gy >= 0) & (gy < self.grid.height)
            obstacle_distance = np.full(self.count, max_range)
            obstacle_distance[inside] = self.distance[gy[inside], gx[inside]]
            likelihood = np.exp(-0.5 * (obstacle_distance / self.sigma) ** 2) + self.random_probability
            measurement_log += np.log(likelihood)
        # Adjacent LiDAR beams are strongly correlated. Tempering their joint
        # likelihood prevents one imperfect-map match from taking all mass.
        log_weights += self.measurement_power * measurement_log
        log_weights -= log_weights.max()
        self.weights = np.exp(log_weights)
        total = self.weights.sum()
        self.weights[:] = self.weights / total if total > 0 else 1.0 / self.count

    def effective_sample_size(self):
        return 1.0 / np.sum(self.weights ** 2)

    def resample(self, roughening=(0.0, 0.0), random_injection=0.0):
        positions = (self.rng.random() + np.arange(self.count)) / self.count
        indexes = np.searchsorted(np.cumsum(self.weights), positions)
        self.particles = self.particles[indexes].copy()
        position_std, heading_std = roughening
        if position_std > 0.0:
            self.particles[:, :2] += self.rng.normal(
                0.0, position_std, size=(self.count, 2)
            )
        if heading_std > 0.0:
            self.particles[:, 2] += self.rng.normal(0.0, heading_std, self.count)
            self.particles[:, 2] = (
                self.particles[:, 2] + math.pi
            ) % (2.0 * math.pi) - math.pi
        injection_count = min(
            self.count, max(0, int(round(float(random_injection) * self.count)))
        )
        if injection_count:
            injected = self.rng.choice(self.count, size=injection_count, replace=False)
            self.particles[injected] = self._sample_free(injection_count)
        self.weights.fill(1.0 / self.count)

    def estimate(self):
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        s = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        c = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        return float(x), float(y), wrap_angle(math.atan2(s, c))
