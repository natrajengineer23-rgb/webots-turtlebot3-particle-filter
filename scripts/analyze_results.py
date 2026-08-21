"""Calculate reproducible summary metrics from mapping and localisation logs."""
import argparse
import csv
import math
from pathlib import Path
import statistics


DEFAULT_PROJECT = Path(__file__).resolve().parents[1]


def read_rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def percentile(values, fraction):
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def path_length(rows):
    points = [(float(row["x_m"]), float(row["y_m"])) for row in rows]
    return sum(math.hypot(bx - ax, by - ay)
               for (ax, ay), (bx, by) in zip(points, points[1:]))


def separation(first, last):
    return math.hypot(float(last["x_m"]) - float(first["x_m"]),
                      float(last["y_m"]) - float(first["y_m"]))


def calculate(project):
    mapping = read_rows(project / "results/scan_matching_log.csv")
    localisation = read_rows(project / "results/localization_log.csv")
    if not mapping or not localisation:
        raise ValueError("Both mapping and localisation logs must contain data.")

    residuals = [float(row["icp_rmse_m"]) for row in mapping]
    accepted = [int(row["accepted"]) for row in mapping]
    neff = [float(row["neff"]) for row in localisation]
    particles = int(localisation[0]["particles"])
    threshold = 0.35 * particles
    post_initialisation = [row for row in localisation if float(row["time_s"]) >= 10.0]

    return [
        ("Mapping duration", float(mapping[-1]["time_s"]), "s"),
        ("Mapping updates", len(mapping), "updates"),
        ("Estimated mapping path length", path_length(mapping), "m"),
        ("Accepted ICP matches", sum(accepted) / len(accepted), "fraction"),
        ("Mean ICP RMSE", statistics.fmean(residuals), "m"),
        ("Median ICP RMSE", statistics.median(residuals), "m"),
        ("95th-percentile ICP RMSE", percentile(residuals, 0.95), "m"),
        ("Maximum ICP RMSE", max(residuals), "m"),
        ("Localisation duration", float(localisation[-1]["time_s"]), "s"),
        ("Localisation updates", len(localisation), "updates"),
        ("Estimated localisation path length", path_length(localisation), "m"),
        ("Mean effective sample size", statistics.fmean(neff), "particles"),
        ("Median effective sample size", statistics.median(neff), "particles"),
        ("Minimum effective sample size", min(neff), "particles"),
        ("Resampling-trigger updates", sum(value < threshold for value in neff), "updates"),
        ("Resampling-trigger proportion", sum(value < threshold for value in neff) / len(neff), "fraction"),
        ("Estimated start-finish separation", separation(localisation[0], localisation[-1]), "m"),
        ("Post-initialisation start-finish separation", separation(post_initialisation[0], post_initialisation[-1]), "m"),
    ]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-dir", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = arguments()
    project = args.project_dir.resolve()
    output = args.output or project / "results/metrics_summary.csv"
    metrics = calculate(project)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value", "unit"])
        writer.writerows((name, f"{value:.9g}", unit) for name, value, unit in metrics)
    print(f"Saved {output}")
    for name, value, unit in metrics:
        print(f"{name}: {value:.4g} {unit}")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, KeyError, IndexError, ValueError) as error:
        raise SystemExit(f"ERROR: {error}")
