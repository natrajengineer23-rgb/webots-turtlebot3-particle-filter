"""Create report-ready figures from mapping and localisation outputs."""
import argparse
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

DEFAULT_PROJECT = Path(__file__).resolve().parents[1]


def read_csv(path):
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return {name: np.array([float(row[name]) for row in rows]) for name in rows[0]} if rows else {}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-dir", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def map_plot_bounds(pixels, metadata, padding=0.35):
    """Return world-coordinate bounds around observed map cells."""
    known_rows, known_cols = np.nonzero(pixels != 205)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    height, width = pixels.shape
    full = (origin_x, origin_x + width * resolution,
            origin_y, origin_y + height * resolution)
    if known_rows.size == 0:
        return full
    x_min = origin_x + known_cols.min() * resolution - padding
    x_max = origin_x + (known_cols.max() + 1) * resolution + padding
    y_min = origin_y + (height - known_rows.max() - 1) * resolution - padding
    y_max = origin_y + (height - known_rows.min()) * resolution + padding
    return x_min, x_max, y_min, y_max


def percentile(values, percentage):
    return float(np.percentile(values, percentage))


def main():
    args = arguments()
    project = args.project_dir.resolve()
    map_path = project / "maps/map.pgm"
    scan_log_path = project / "results/scan_matching_log.csv"
    localisation_path = project / "results/localization_log.csv"
    if not map_path.exists():
        raise FileNotFoundError("Run mapping_controller and save the map before plotting.")

    metadata = yaml.safe_load((project / "maps/map.yaml").read_text())
    pixels = np.asarray(Image.open(map_path).convert("L"))
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    height, width = pixels.shape
    extent = [origin_x, origin_x + width * resolution,
              origin_y, origin_y + height * resolution]

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.7), constrained_layout=True)
    figure.suptitle("Odometry-free LiDAR mapping and Monte Carlo localisation", fontsize=15)
    axes[0].imshow(pixels, cmap="gray", origin="upper", extent=extent,
                   vmin=0, vmax=255)
    axes[0].set_title("(a) Occupancy map and MCL path")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    x_min, x_max, y_min, y_max = map_plot_bounds(pixels, metadata)
    axes[0].set_xlim(x_min, x_max)
    axes[0].set_ylim(y_min, y_max)
    axes[0].set_aspect("equal")

    if localisation_path.exists():
        data = read_csv(localisation_path)
        axes[0].plot(data["x_m"], data["y_m"], color="#0072b2", linewidth=1.8,
                     label="MCL estimate")
        axes[0].scatter(data["x_m"][0], data["y_m"][0], color="#009e73",
                        edgecolor="white", linewidth=0.6, label="start", zorder=3)
        axes[0].scatter(data["x_m"][-1], data["y_m"][-1], color="#d55e00",
                        edgecolor="white", linewidth=0.6, label="finish", zorder=3)
        axes[0].legend(loc="upper right", fontsize=8)

        neff = data["neff"]
        particles = int(data["particles"][0])
        threshold = 0.35 * particles
        below = neff < threshold
        axes[2].plot(data["time_s"], neff, color="#0072b2", linewidth=1.1,
                     label=r"$N_{eff}$")
        axes[2].scatter(data["time_s"][below], neff[below], color="#d55e00", s=9,
                        label="resampling trigger", zorder=3)
        axes[2].axhline(threshold, color="#d55e00", linestyle="--", linewidth=1,
                        label=f"threshold = {threshold:.0f}")
        axes[2].set_title("(c) Particle-filter stability")
        axes[2].set_xlabel("time (s)")
        axes[2].set_ylabel(r"effective sample size $N_{eff}$")
        axes[2].set_ylim(0, particles * 1.04)
        axes[2].grid(alpha=0.25)
        axes[2].legend(fontsize=8, loc="lower right")
        axes[2].text(
            0.02, 0.97,
            f"median $N_{{eff}}$: {np.median(neff):.0f}\n"
            f"below threshold: {below.sum()}/{len(neff)} ({100 * below.mean():.1f}%)",
            transform=axes[2].transAxes, va="top", fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "0.75"},
        )
    else:
        axes[0].text(0.5, 0.05, "Run localisation to add the trajectory",
                     transform=axes[0].transAxes, ha="center")
        axes[2].text(0.5, 0.5, "Run localisation to generate diagnostics",
                     ha="center", va="center")
        axes[2].set_axis_off()

    if scan_log_path.exists():
        scan = read_csv(scan_log_path)
        residual_cm = scan["icp_rmse_m"] * 100.0
        mean_cm = residual_cm.mean()
        p95_cm = percentile(residual_cm, 95)
        accepted = scan["accepted"] > 0.5
        axes[1].plot(scan["time_s"], residual_cm, color="#7f3c8d", linewidth=1.0)
        axes[1].axhline(mean_cm, color="#009e73", linestyle="--", linewidth=1.2,
                        label=f"mean = {mean_cm:.2f} cm")
        axes[1].axhline(p95_cm, color="#e69f00", linestyle=":", linewidth=1.2,
                        label=f"95th percentile = {p95_cm:.2f} cm")
        axes[1].set_title("(b) Scan-matching residual")
        axes[1].set_xlabel("time (s)")
        axes[1].set_ylabel("ICP RMSE (cm)")
        axes[1].grid(alpha=0.25)
        axes[1].legend(fontsize=8)
        axes[1].text(
            0.98, 0.97,
            f"accepted: {accepted.sum()}/{len(accepted)} ({100 * accepted.mean():.1f}%)\n"
            f"maximum: {residual_cm.max():.2f} cm",
            transform=axes[1].transAxes, ha="right", va="top", fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "0.75"},
        )
    else:
        axes[1].text(0.5, 0.5, "Run mapping to generate diagnostics",
                     ha="center", va="center")
        axes[1].set_axis_off()

    output = args.output or project / "results/report_results.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=220, bbox_inches="tight")
    print(f"Saved {output}")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, KeyError, IndexError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
