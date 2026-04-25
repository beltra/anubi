import importlib.util
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUNNER = PACKAGE_ROOT / "sim_scenarios" / "safety_regression_runner.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("safety_regression_runner", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SafetyRegressionRunnerTest(unittest.TestCase):

    def test_synthetic_case_passes_expected_levels(self):
        runner = load_runner()
        case = {
            "name": "stale_sensor",
            "expected_diagnostics": [
                {"name": "anubi/topic/lidar_points_undistorted", "min_level": "ERROR"}
            ],
            "sample_diagnostics": [
                {"name": "anubi/topic/lidar_points_undistorted", "level": "ERROR"}
            ],
        }

        result = runner.evaluate_synthetic_case(case)

        self.assertTrue(result.passed)
        self.assertEqual(result.checks[0].observed, "ERROR")

    def test_synthetic_case_fails_missing_diagnostic(self):
        runner = load_runner()
        case = {
            "name": "missing",
            "expected_diagnostics": [
                {"name": "anubi/topic/cmd_vel_safe", "max_level": "OK"}
            ],
            "sample_diagnostics": [],
        }

        result = runner.evaluate_synthetic_case(case)

        self.assertFalse(result.passed)
        self.assertEqual(result.checks[0].observed, "missing")

    def test_command_source_expectation(self):
        runner = load_runner()
        case = {
            "name": "operator_override",
            "expected_command_source": "joystick_override",
            "sample_command_source": "nav",
        }

        result = runner.evaluate_synthetic_case(case)

        self.assertFalse(result.passed)
        self.assertEqual(result.checks[0].name, "command_source")


if __name__ == "__main__":
    unittest.main()
