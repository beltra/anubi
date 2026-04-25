#!/usr/bin/env python3
"""Run safety regression checks and write JSON pass/fail reports."""

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def diagnostic_level(value) -> int:
    if isinstance(value, bytes):
        return int.from_bytes(value, byteorder="little")
    return int(value)


LEVELS = {
    "OK": diagnostic_level(DiagnosticStatus.OK),
    "WARN": diagnostic_level(DiagnosticStatus.WARN),
    "ERROR": diagnostic_level(DiagnosticStatus.ERROR),
}


@dataclass
class CheckResult:
    name: str
    passed: bool
    expected: str
    observed: str


@dataclass
class CaseResult:
    name: str
    passed: bool
    mode: str
    checks: List[CheckResult]


def level_name(level: int) -> str:
    for name, value in LEVELS.items():
        if int(level) == value:
            return name
    return str(level)


def resolve_manifest(path_or_name: str) -> str:
    if os.path.isabs(path_or_name):
        return path_or_name
    name = path_or_name if path_or_name.endswith(".yaml") else path_or_name + ".yaml"
    share = get_package_share_directory("sim_scenarios")
    return os.path.join(share, "scenarios", name)


def load_manifest(path_or_name: str) -> Dict:
    path = resolve_manifest(path_or_name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Safety regression manifest not found: {path}")
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def diagnostics_from_samples(samples: List[Dict]) -> Dict[str, int]:
    diagnostics = {}
    for item in samples:
        diagnostics[str(item["name"])] = LEVELS[str(item["level"]).upper()]
    return diagnostics


def evaluate_diagnostic_expectations(case: Dict, diagnostics: Dict[str, int]) -> List[CheckResult]:
    checks = []
    for expected in case.get("expected_diagnostics", []):
        name = str(expected["name"])
        observed = diagnostics.get(name)
        if observed is None:
            checks.append(CheckResult(name, False, "diagnostic present", "missing"))
            continue

        if "min_level" in expected:
            threshold = LEVELS[str(expected["min_level"]).upper()]
            passed = observed >= threshold
            checks.append(
                CheckResult(
                    name,
                    passed,
                    f">= {level_name(threshold)}",
                    level_name(observed),
                )
            )
        elif "max_level" in expected:
            threshold = LEVELS[str(expected["max_level"]).upper()]
            passed = observed <= threshold
            checks.append(
                CheckResult(
                    name,
                    passed,
                    f"<= {level_name(threshold)}",
                    level_name(observed),
                )
            )
        else:
            exact = LEVELS[str(expected["level"]).upper()]
            passed = observed == exact
            checks.append(
                CheckResult(name, passed, level_name(exact), level_name(observed))
            )
    return checks


def evaluate_command_source(case: Dict, observed_source: str) -> List[CheckResult]:
    expected = case.get("expected_command_source", "")
    if not expected:
        return []
    return [
        CheckResult(
            "command_source",
            observed_source == expected,
            str(expected),
            observed_source or "missing",
        )
    ]


def evaluate_synthetic_case(case: Dict) -> CaseResult:
    diagnostics = diagnostics_from_samples(case.get("sample_diagnostics", []))
    checks = evaluate_diagnostic_expectations(case, diagnostics)
    checks.extend(evaluate_command_source(case, str(case.get("sample_command_source", ""))))
    return CaseResult(
        name=str(case["name"]),
        passed=all(check.passed for check in checks),
        mode="synthetic",
        checks=checks,
    )


class LiveSafetyRegressionRunner(Node):

    def __init__(self, timeout_sec: float):
        super().__init__("safety_regression_runner")
        self._timeout_sec = timeout_sec
        self._diagnostics: Dict[str, int] = {}
        self._command_source = ""
        self._override_pub = self.create_publisher(Bool, "/joystick_override", 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_collision_free", 10)
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )
        self._command_source_sub = self.create_subscription(
            String, "/safety/command_source", self._on_command_source, 10
        )

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            self._diagnostics[status.name] = diagnostic_level(status.level)

    def _on_command_source(self, msg: String) -> None:
        self._command_source = msg.data

    def run_case(self, case: Dict) -> CaseResult:
        start = time.monotonic()
        override_active = bool(case.get("publish_joystick_override", False))
        publish_cmd = bool(case.get("publish_nav_command", False))

        while time.monotonic() - start < self._timeout_sec:
            if override_active:
                msg = Bool()
                msg.data = True
                self._override_pub.publish(msg)
            if publish_cmd:
                cmd = Twist()
                cmd.linear.x = float(case.get("nav_command_linear_x", 0.1))
                self._cmd_pub.publish(cmd)

            rclpy.spin_once(self, timeout_sec=0.1)
            checks = evaluate_diagnostic_expectations(case, self._diagnostics)
            checks.extend(evaluate_command_source(case, self._command_source))
            if checks and all(check.passed for check in checks):
                self._clear_override_if_needed(override_active)
                return CaseResult(str(case["name"]), True, "live", checks)

        checks = evaluate_diagnostic_expectations(case, self._diagnostics)
        checks.extend(evaluate_command_source(case, self._command_source))
        self._clear_override_if_needed(override_active)
        return CaseResult(str(case["name"]), False, "live", checks)

    def _clear_override_if_needed(self, override_active: bool) -> None:
        if not override_active:
            return
        msg = Bool()
        msg.data = False
        for _ in range(3):
            self._override_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def select_cases(manifest: Dict, names: Optional[List[str]]) -> List[Dict]:
    cases = list(manifest.get("cases", []))
    if not names:
        return cases
    selected = [case for case in cases if str(case.get("name")) in names]
    missing = sorted(set(names) - {str(case.get("name")) for case in selected})
    if missing:
        raise RuntimeError(f"Unknown safety regression case(s): {', '.join(missing)}")
    return selected


def write_report(path: str, mode: str, results: List[CaseResult]) -> None:
    payload = {
        "generated_at_unix": time.time(),
        "mode": mode,
        "summary": {
            "cases": len(results),
            "passed": sum(1 for result in results if result.passed),
            "failed": sum(1 for result in results if not result.passed),
        },
        "results": [
            {
                "name": result.name,
                "passed": result.passed,
                "mode": result.mode,
                "checks": [asdict(check) for check in result.checks],
            }
            for result in results
        ],
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        default="safety_regression_cases.yaml",
        help="Safety regression manifest name or absolute path",
    )
    parser.add_argument("--report-file", default="")
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--case", action="append", dest="cases", default=[])
    args = parser.parse_args(argv)

    manifest = load_manifest(args.manifest)
    cases = select_cases(manifest, args.cases)
    mode = "synthetic" if args.synthetic else "live"
    report = args.report_file or f"/tmp/safety_regression_{mode}_{int(time.time())}.json"

    if args.synthetic:
        results = [evaluate_synthetic_case(case) for case in cases]
    else:
        rclpy.init()
        runner = LiveSafetyRegressionRunner(timeout_sec=args.timeout)
        try:
            results = [runner.run_case(case) for case in cases]
        finally:
            runner.destroy_node()
            rclpy.shutdown()

    write_report(report, mode, results)
    print(f"Safety regression report written to: {report}")

    if any(not result.passed for result in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main(sys.argv[1:])
