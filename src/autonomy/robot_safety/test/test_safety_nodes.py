import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]


def load_script(name):
    path = SCRIPTS_DIR / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


joystick = load_script("joystick_safety_node.py")
tilt = load_script("tilt_monitor_node.py")
limiter = load_script("velocity_limiter_node.py")


def test_joystick_active_uses_axes_and_keys():
    neutral = SimpleNamespace(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0)
    assert joystick.joystick_active(neutral, 0.05) is False

    axis_active = SimpleNamespace(lx=0.0, ly=0.06, rx=0.0, ry=0.0, keys=0)
    assert joystick.joystick_active(axis_active, 0.05) is True

    key_active = SimpleNamespace(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0x0004)
    assert joystick.joystick_active(key_active, 0.05) is True


def test_tilt_roll_pitch_and_recovery_ack():
    half_angle = math.radians(30.0) / 2.0
    q = SimpleNamespace(x=math.sin(half_angle), y=0.0, z=0.0, w=math.cos(half_angle))
    roll, pitch = tilt.TiltMonitor._roll_pitch(q)
    assert math.degrees(roll) == pytest_approx(30.0)
    assert math.degrees(pitch) == pytest_approx(0.0)

    clear_tilt = math.radians(15.0)
    assert tilt.should_clear_tilt_estop(True, math.radians(10.0), clear_tilt, 0x0008, 0x0008)
    assert not tilt.should_clear_tilt_estop(True, math.radians(20.0), clear_tilt, 0x0008, 0x0008)
    assert not tilt.should_clear_tilt_estop(True, math.radians(10.0), clear_tilt, 0, 0x0008)
    assert tilt.parse_key_mask("0x0008") == 8


def test_velocity_limiter_priority_order():
    fresh_sources = {"nav": "fresh", "manual": "fresh", "recovery": "fresh"}
    assert limiter.select_command_source(True, True, True, fresh_sources) == "estop"
    assert limiter.select_command_source(False, True, True, fresh_sources) == "tilt_estop"
    assert limiter.select_command_source(False, False, True, fresh_sources) == "joystick_override"
    assert limiter.select_command_source(False, False, False, {}) == "waiting_for_cmd"
    assert limiter.select_command_source(False, False, False, {"nav": "stale"}) == "watchdog"
    assert limiter.select_command_source(False, False, False, {"nav": "fresh"}) == "nav"
    assert limiter.select_command_source(False, False, False, fresh_sources) == "recovery"


def test_real_launch_and_script_default_to_safety():
    launch = (
        WORKSPACE_ROOT
        / "src"
        / "platform"
        / "go2_bringup"
        / "launch"
        / "go2_bringup.launch.py"
    ).read_text(encoding="utf-8")
    script = (WORKSPACE_ROOT / "scripts" / "run_go2_real.sh").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_safety",   default_value="true"' in launch
    assert 'USE_SAFETY="true"' in script
    assert "--no-safety" in script
    assert "recovery_ack_key_mask" in launch
    assert "recovery_ack_key_mask" in script


def pytest_approx(value):
    import pytest

    return pytest.approx(value, abs=0.001)
