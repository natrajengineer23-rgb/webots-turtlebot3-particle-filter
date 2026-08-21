"""Odometry-free TurtleBot3 Monte Carlo localisation controller."""
import csv
import sys
from pathlib import Path

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from controller import Keyboard, Robot
from src.geometry import wheel_commands
from src.occupancy_grid import OccupancyGrid
from src.particle_filter import ParticleFilter


def requested_twist(key, keyboard, robot_cfg):
    if key in (keyboard.UP, ord("W"), ord("w")):
        return robot_cfg["forward_speed"], 0.0
    if key in (keyboard.DOWN, ord("S"), ord("s")):
        return -robot_cfg["reverse_speed"], 0.0
    if key in (keyboard.LEFT, ord("A"), ord("a")):
        return 0.0, robot_cfg["turn_speed"]
    if key in (keyboard.RIGHT, ord("D"), ord("d")):
        return 0.0, -robot_cfg["turn_speed"]
    return 0.0, 0.0


def main():
    cfg = yaml.safe_load((PROJECT / "config/default.yaml").read_text())
    robot_cfg, mapping_cfg, localisation_cfg = cfg["robot"], cfg["mapping"], cfg["localization"]
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    lidar = robot.getDevice("LDS-01")
    for motor in (left_motor, right_motor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    lidar.enable(timestep)
    keyboard = Keyboard()
    keyboard.enable(timestep)

    map_path = PROJECT / "maps/map.yaml"
    if not map_path.exists():
        raise FileNotFoundError("maps/map.yaml is missing. Run mapping_controller first and press M.")
    grid = OccupancyGrid.load(map_path)
    particle_filter = ParticleFilter(
        grid,
        count=localisation_cfg["particles"],
        sensor_sigma=localisation_cfg["sensor_sigma"],
        random_probability=localisation_cfg["random_measurement_probability"],
        seed=localisation_cfg["seed"],
        measurement_power=localisation_cfg["measurement_power"],
    )
    results_dir = PROJECT / "results"
    results_dir.mkdir(exist_ok=True)
    log_handle = (results_dir / "localization_log.csv").open("w", newline="")
    log = csv.writer(log_handle)
    log.writerow(["time_s", "x_m", "y_m", "yaw_rad", "neff", "particles", "linear_command", "angular_command"])

    previous_linear = previous_angular = 0.0
    update_count = 0
    max_range = min(mapping_cfg["max_range"], lidar.getMaxRange())
    print("ODOMETRY-FREE MONTE CARLO LOCALISATION STARTED")
    print("Arrows/WASD: drive | Q: exit")

    while robot.step(timestep) != -1:
        if previous_linear != 0.0 or previous_angular != 0.0:
            particle_filter.predict_command(
                previous_linear, previous_angular, dt, localisation_cfg["motion_noise"]
            )

        if update_count % localisation_cfg["update_every_steps"] == 0:
            ranges = np.asarray(lidar.getRangeImage(), dtype=float)
            angles = np.linspace(lidar.getFov() / 2.0, -lidar.getFov() / 2.0, len(ranges))
            particle_filter.update(ranges, angles, max_range, localisation_cfg["beam_step"])
            neff = particle_filter.effective_sample_size()
            estimate = particle_filter.estimate()
            log.writerow([
                robot.getTime(), *estimate, neff, particle_filter.count, previous_linear, previous_angular
            ])
            log_handle.flush()
            if neff < localisation_cfg["resample_threshold"] * particle_filter.count:
                particle_filter.resample(
                    localisation_cfg["resample_roughening"],
                    localisation_cfg["random_particle_injection"],
                )
            if update_count % (localisation_cfg["update_every_steps"] * 20) == 0:
                print(f"estimate x={estimate[0]:.2f}, y={estimate[1]:.2f}, yaw={estimate[2]:.2f} rad, Neff={neff:.0f}")

        key = keyboard.getKey()
        if key in (ord("Q"), ord("q")):
            break
        linear, angular = requested_twist(key, keyboard, robot_cfg)
        left, right = wheel_commands(
            linear, angular, robot_cfg["wheel_radius"], robot_cfg["axle_length"], robot_cfg["max_motor_speed"]
        )
        left_motor.setVelocity(left)
        right_motor.setVelocity(right)
        previous_linear, previous_angular = linear, angular
        update_count += 1

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    log_handle.close()


if __name__ == "__main__":
    main()
