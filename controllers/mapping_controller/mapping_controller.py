"""Odometry-free TurtleBot3 teleoperation and LiDAR mapping controller."""
from collections import deque
import csv
import math
import sys
from pathlib import Path

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from controller import Keyboard, Robot
from src.geometry import compose_pose, relative_pose, unicycle_increment, wheel_commands
from src.occupancy_grid import OccupancyGrid
from src.scan_matching import icp_2d, pose_matrix, ranges_to_points, transform_points


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


def plausible(result, previous_pose, predicted_pose, mapping_cfg):
    step = relative_pose(previous_pose, result.increment)
    correction = relative_pose(predicted_pose, result.increment)
    return (
        result.converged
        and result.rmse <= mapping_cfg["reject_rmse_above"]
        and math.hypot(step[0], step[1]) <= mapping_cfg["reject_translation_above"]
        and abs(step[2]) <= mapping_cfg["reject_rotation_above"]
        and math.hypot(correction[0], correction[1]) <= mapping_cfg["reject_correction_translation_above"]
        and abs(correction[2]) <= mapping_cfg["reject_correction_rotation_above"]
    )


def main():
    cfg = yaml.safe_load((PROJECT / "config/default.yaml").read_text())
    robot_cfg, mapping_cfg = cfg["robot"], cfg["mapping"]
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

    grid = OccupancyGrid(
        mapping_cfg["width"], mapping_cfg["height"], mapping_cfg["resolution"], mapping_cfg["origin"],
        mapping_cfg["log_odds_free"], mapping_cfg["log_odds_occupied"],
        mapping_cfg["log_odds_min"], mapping_cfg["log_odds_max"],
    )
    results_dir = PROJECT / "results"
    results_dir.mkdir(exist_ok=True)
    log_handle = (results_dir / "scan_matching_log.csv").open("w", newline="")
    log = csv.writer(log_handle)
    log.writerow(["time_s", "x_m", "y_m", "yaw_rad", "icp_rmse_m", "pairs", "iterations", "accepted"])

    pose = (0.0, 0.0, 0.0)
    local_submap = deque(maxlen=mapping_cfg["submap_scans"])
    command_increment = (0.0, 0.0, 0.0)
    previous_linear = previous_angular = 0.0
    update_count = 0
    max_range = min(mapping_cfg["max_range"], lidar.getMaxRange())
    print("ODOMETRY-FREE MAPPING STARTED")
    print("Arrows/WASD: drive | M: save map | Q: save and exit")

    while robot.step(timestep) != -1:
        command_increment = compose_pose(
            command_increment,
            unicycle_increment(previous_linear, previous_angular, dt),
        )
        if update_count % mapping_cfg["update_every_steps"] == 0:
            ranges = np.asarray(lidar.getRangeImage(), dtype=float)
            angles = np.linspace(lidar.getFov() / 2.0, -lidar.getFov() / 2.0, len(ranges))
            current_scan = ranges_to_points(ranges, angles, max_range, step=mapping_cfg["scan_step"])
            accepted, rmse, pairs, iterations = True, 0.0, len(current_scan), 0
            predicted_pose = compose_pose(pose, command_increment)
            if local_submap:
                reference = np.concatenate(tuple(local_submap), axis=0)
                result = icp_2d(
                    current_scan, reference,
                    max_iterations=mapping_cfg["icp_max_iterations"],
                    tolerance=mapping_cfg["icp_tolerance"],
                    max_correspondence=mapping_cfg["icp_max_correspondence"],
                    trim_fraction=mapping_cfg["icp_trim_fraction"],
                    min_pairs=mapping_cfg["icp_min_pairs"],
                    initial_guess=predicted_pose,
                )
                accepted = plausible(result, pose, predicted_pose, mapping_cfg)
                rmse, pairs, iterations = result.rmse, result.correspondences, result.iterations
                if accepted:
                    pose = result.increment
                else:
                    pose = predicted_pose
                    correction = relative_pose(predicted_pose, result.increment)
                    print(
                        f"ICP rejected: RMSE={rmse:.3f} m, pairs={pairs}, "
                        f"correction={math.hypot(correction[0], correction[1]):.3f} m/"
                        f"{math.degrees(correction[2]):.1f} deg"
                    )
            else:
                pose = predicted_pose
            if accepted:
                grid.update_scan(pose, ranges, angles, max_range)
                local_submap.append(transform_points(current_scan, pose_matrix(pose)))
            command_increment = (0.0, 0.0, 0.0)
            log.writerow([robot.getTime(), *pose, rmse, pairs, iterations, int(accepted)])
            log_handle.flush()
            if update_count % (mapping_cfg["update_every_steps"] * 20) == 0:
                print(f"pose x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={math.degrees(pose[2]):.1f} deg, ICP={rmse:.3f} m")

        key = keyboard.getKey()
        if key in (ord("M"), ord("m"), ord("Q"), ord("q")):
            grid.save(PROJECT / "maps")
            print("Saved maps/map.pgm and maps/map.yaml")
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
    grid.save(PROJECT / "maps")
    log_handle.close()


if __name__ == "__main__":
    main()
