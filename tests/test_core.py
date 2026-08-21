import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from src.geometry import compose_pose, relative_pose, unicycle_increment, wheel_commands, wrap_angle
from src.occupancy_grid import OccupancyGrid, bresenham
from src.particle_filter import ParticleFilter, distance_field
from src.scan_matching import icp_2d, pose_matrix, ranges_to_points, transform_points
from scripts.prepare_and_run_webots import (
    CONTROLLERS,
    set_controller,
    webots_launch_command,
    write_runtime_files,
)


class GeometryTests(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertTrue(math.isclose(wrap_angle(3 * math.pi), -math.pi))

    def test_unicycle_straight_and_turn(self):
        self.assertTrue(np.allclose(unicycle_increment(0.2, 0.0, 0.5), (0.1, 0.0, 0.0)))
        increment = unicycle_increment(0.2, 1.0, 0.5)
        self.assertTrue(np.allclose(compose_pose((0, 0, 0), increment), increment))

    def test_relative_pose_inverts_composition(self):
        reference = (1.0, -0.5, 0.7)
        increment = (0.2, -0.1, -0.3)
        self.assertTrue(np.allclose(relative_pose(reference, compose_pose(reference, increment)), increment))

    def test_wheel_commands_are_actuation_only(self):
        left, right = wheel_commands(0.1, 0.0, 0.05, 0.2, 10.0)
        self.assertAlmostEqual(left, right)
        left, right = wheel_commands(0.0, 1.0, 0.05, 0.2, 10.0)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)


class MappingTests(unittest.TestCase):
    def test_bresenham_endpoints(self):
        cells = bresenham(0, 0, 3, 2)
        self.assertEqual(cells[0], (0, 0))
        self.assertEqual(cells[-1], (3, 2))

    def test_scan_marks_free_and_occupied(self):
        grid = OccupancyGrid(20, 20, 0.1, (-1, -1))
        grid.update_scan((0, 0, 0), np.array([0.5]), np.array([0.0]), 1.0)
        self.assertGreater(grid.log_odds[10, 15], 0)
        self.assertLess(grid.log_odds[10, 12], 0)

    def test_map_round_trip(self):
        grid = OccupancyGrid(10, 8, 0.1, (-0.5, -0.4))
        grid.log_odds.fill(-4.0)
        grid.log_odds[2:4, 5] = 4.0
        with tempfile.TemporaryDirectory() as directory:
            grid.save(directory)
            loaded = OccupancyGrid.load(Path(directory) / "map.yaml")
            self.assertEqual(loaded.log_odds.shape, grid.log_odds.shape)
            self.assertGreater(loaded.log_odds[2, 5], 0.0)


class ScanMatchingTests(unittest.TestCase):
    def test_range_conversion_filters_invalid_values(self):
        points = ranges_to_points([1.0, float("inf"), 0.05], [0.0, 1.0, 2.0], 3.5, step=1)
        self.assertEqual(points.shape, (1, 2))
        self.assertTrue(np.allclose(points[0], (1.0, 0.0)))

    def test_icp_recovers_known_motion(self):
        rng = np.random.default_rng(4)
        target = rng.uniform((-1.5, -1.0), (1.5, 1.0), size=(180, 2))
        yaw, translation = 0.045, np.array([0.07, -0.025])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        source = (target - translation) @ rotation
        result = icp_2d(source, target, max_correspondence=0.25, trim_fraction=1.0)
        self.assertTrue(result.converged)
        self.assertTrue(np.allclose(result.increment[:2], translation, atol=0.01))
        self.assertAlmostEqual(result.increment[2], yaw, delta=0.01)
        self.assertLess(result.rmse, 0.01)

    def test_icp_uses_world_frame_initial_guess(self):
        rng = np.random.default_rng(12)
        source = rng.uniform((-1.0, -0.8), (1.0, 0.8), size=(160, 2))
        true_pose = (1.2, -0.7, 0.25)
        target = transform_points(source, pose_matrix(true_pose))
        result = icp_2d(
            source,
            target,
            max_correspondence=0.25,
            trim_fraction=1.0,
            initial_guess=(1.15, -0.65, 0.22),
        )
        self.assertTrue(result.converged)
        self.assertTrue(np.allclose(result.increment, true_pose, atol=0.01))


class ParticleFilterTests(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid(30, 30, 0.1, (-1.5, -1.5))
        self.grid.log_odds.fill(-4.0)
        self.grid.log_odds[:, 0] = 4.0
        self.grid.log_odds[:, -1] = 4.0
        self.grid.log_odds[0, :] = 4.0
        self.grid.log_odds[-1, :] = 4.0

    def test_distance_field(self):
        occupied = np.zeros((5, 5), dtype=bool)
        occupied[2, 2] = True
        field = distance_field(occupied)
        self.assertEqual(field[2, 2], 0)
        self.assertEqual(field[2, 3], 1)

    def test_command_prediction_without_encoders(self):
        particle_filter = ParticleFilter(self.grid, count=40, seed=2)
        particle_filter.particles[:] = (0.0, 0.0, 0.0)
        particle_filter.predict_command(0.2, 0.0, 0.5, (0.0, 0.0))
        self.assertTrue(np.allclose(particle_filter.particles[:, 0], 0.1))
        self.assertTrue(np.allclose(particle_filter.particles[:, 1], 0.0))

    def test_systematic_resampling_restores_uniform_weights(self):
        particle_filter = ParticleFilter(self.grid, count=20, seed=3)
        particle_filter.weights[:] = 0.0
        particle_filter.weights[0] = 1.0
        particle_filter.resample()
        self.assertTrue(np.allclose(particle_filter.weights, 1.0 / 20))

    def test_resampling_roughens_and_injects_particles(self):
        particle_filter = ParticleFilter(self.grid, count=100, seed=11)
        particle_filter.particles[:] = (0.0, 0.0, 0.0)
        particle_filter.weights[:] = 1.0 / particle_filter.count
        particle_filter.resample(roughening=(0.01, 0.02), random_injection=0.10)
        self.assertGreater(np.std(particle_filter.particles[:, 0]), 0.01)
        self.assertGreater(np.std(particle_filter.particles[:, 2]), 0.02)

    def test_measurement_tempering_reduces_weight_collapse(self):
        wall_x = int((1.0 - self.grid.origin[0]) / self.grid.resolution)
        self.grid.log_odds[:, wall_x] = 4.0
        sharp = ParticleFilter(self.grid, count=40, sensor_sigma=0.10, seed=13)
        tempered = ParticleFilter(
            self.grid, count=40, sensor_sigma=0.10, seed=13, measurement_power=0.25
        )
        particles = np.column_stack((
            np.linspace(-0.5, 0.3, 40), np.zeros(40), np.zeros(40)
        ))
        sharp.particles[:] = particles
        tempered.particles[:] = particles
        ranges = np.full(12, 1.0)
        angles = np.zeros(12)
        sharp.update(ranges, angles, max_range=3.5, beam_step=1)
        tempered.update(ranges, angles, max_range=3.5, beam_step=1)
        self.assertGreater(tempered.effective_sample_size(), sharp.effective_sample_size())

    def test_lidar_likelihood_favours_consistent_particle(self):
        wall_x = int((1.0 - self.grid.origin[0]) / self.grid.resolution)
        self.grid.log_odds[:, wall_x] = 4.0
        particle_filter = ParticleFilter(self.grid, count=2, sensor_sigma=0.10, seed=5)
        particle_filter.particles[:] = ((0.0, 0.0, 0.0), (-0.5, 0.0, 0.0))
        particle_filter.weights[:] = 0.5
        particle_filter.update(np.array([1.0]), np.array([0.0]), max_range=3.5, beam_step=1)
        self.assertGreater(particle_filter.weights[0], particle_filter.weights[1])


class LauncherTests(unittest.TestCase):
    def test_runtime_keeps_virtual_environment_interpreter_path(self):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            interpreter = project / ".venv/bin/python3"
            write_runtime_files(project, interpreter)
            expected = f"[python]\nCOMMAND = {interpreter}\n"
            for controller in CONTROLLERS.values():
                runtime = project / "controllers" / controller / "runtime.ini"
                self.assertEqual(runtime.read_text(), expected)

    def test_macos_launcher_uses_application_bundle(self):
        command = webots_launch_command(
            Path("/Applications/Webots.app"), Path("/tmp/project/world.wbt")
        )
        self.assertEqual(command[:4], ["/usr/bin/open", "-n", "-a", "/Applications/Webots.app"])
        self.assertIn("/tmp/project/world.wbt", command)
        self.assertEqual(command[-2:], ["--args", "--mode=realtime"])

    def test_controller_is_replaced(self):
        world = 'TurtleBot3Burger {\n  controller "old_controller"\n}\n'
        configured = set_controller(world, "mapping_controller")
        self.assertIn('controller "mapping_controller"', configured)
        self.assertNotIn("old_controller", configured)

    def test_controller_is_inserted_when_missing(self):
        world = "TurtleBot3Burger {\n  translation 0 0 0\n}\n"
        configured = set_controller(world, "localization_controller")
        self.assertIn('controller "localization_controller"', configured)


if __name__ == "__main__":
    unittest.main()
