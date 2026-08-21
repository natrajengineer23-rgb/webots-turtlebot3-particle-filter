from pathlib import Path
import math
import numpy as np
import yaml
from PIL import Image


def bresenham(x0, y0, x1, y1):
    """Integer grid cells on a line, including both endpoints."""
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


class OccupancyGrid:
    def __init__(self, width, height, resolution, origin, free=-0.4, occupied=0.85,
                 minimum=-5.0, maximum=5.0):
        self.width, self.height = int(width), int(height)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.free, self.occupied = free, occupied
        self.minimum, self.maximum = minimum, maximum

    def world_to_grid(self, x, y):
        return int(math.floor((x - self.origin[0]) / self.resolution)), int(math.floor((y - self.origin[1]) / self.resolution))

    def inside(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update_scan(self, pose, ranges, angles, max_range):
        rx, ry, heading = pose
        rgx, rgy = self.world_to_grid(rx, ry)
        if not self.inside(rgx, rgy):
            return
        for distance, angle in zip(ranges, angles):
            if not np.isfinite(distance) or distance <= 0.02:
                distance, hit = max_range, False
            else:
                hit = distance < max_range * 0.995
                distance = min(float(distance), max_range)
            ex = rx + distance * math.cos(heading + angle)
            ey = ry + distance * math.sin(heading + angle)
            egx, egy = self.world_to_grid(ex, ey)
            cells = bresenham(rgx, rgy, egx, egy)
            for gx, gy in cells[:-1]:
                if self.inside(gx, gy):
                    self.log_odds[gy, gx] += self.free
            gx, gy = cells[-1]
            if hit and self.inside(gx, gy):
                self.log_odds[gy, gx] += self.occupied
        np.clip(self.log_odds, self.minimum, self.maximum, out=self.log_odds)

    def probability(self):
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def save(self, directory, stem="map"):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        probability = self.probability()
        pixels = np.full(probability.shape, 205, dtype=np.uint8)
        pixels[probability < 0.35] = 254
        pixels[probability > 0.65] = 0
        Image.fromarray(np.flipud(pixels), mode="L").save(directory / f"{stem}.pgm")
        metadata = {"image": f"{stem}.pgm", "resolution": self.resolution,
                    "origin": [float(self.origin[0]), float(self.origin[1]), 0.0],
                    "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.35}
        (directory / f"{stem}.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))

    @classmethod
    def load(cls, yaml_path):
        yaml_path = Path(yaml_path)
        metadata = yaml.safe_load(yaml_path.read_text())
        pixels = np.flipud(np.asarray(Image.open(yaml_path.parent / metadata["image"]).convert("L")))
        grid = cls(pixels.shape[1], pixels.shape[0], metadata["resolution"], metadata["origin"][:2])
        probability = np.where(pixels < 65, 0.97, np.where(pixels > 250, 0.03, 0.5))
        grid.log_odds = np.log(probability / (1.0 - probability)).astype(np.float32)
        return grid

