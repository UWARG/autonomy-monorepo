import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def load_script(name):
    path = ROOT / "airside" / "scripts" / name
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


ANALYZER = load_script("analyze_follow_hitl.py")
SELECTOR = load_script("select_follow_hitl_config.py")


def target_row(index, scenario):
    capture = index * 0.05
    confirmed = index % 2 == 0
    detector_sequence = index if confirmed else index - 1
    detector_capture = detector_sequence * 0.05
    return {
        "event": "target",
        "host_time_s": f"{capture + 0.02:.3f}",
        "capture_time_s": f"{capture:.3f}",
        "tracker_capture_time_s": f"{capture:.3f}",
        "detector_capture_time_s": f"{detector_capture:.3f}",
        "ros_receive_s": f"{capture + 0.02:.3f}",
        "detector_to_ros_latency_s": f"{capture + 0.02 - detector_capture:.3f}",
        "sequence_num": str(index),
        "detector_sequence_num": str(detector_sequence),
        "detector_confirmed": "1" if confirmed else "0",
        "inference_age_s": f"{capture - detector_capture:.3f}",
        "track_id": "4",
        "scenario": scenario,
    }


def test_detector_and_tracker_cadence_are_analyzed_separately():
    scenarios = sorted(ANALYZER.REQUIRED_SCENARIOS)
    rows = [target_row(index, scenarios[index % len(scenarios)]) for index in range(80)]
    rows.extend(
        [
            {
                "event": "setpoint",
                "host_time_s": "2.000",
                "sp_vx": "0.5",
                "sp_vy": "0",
                "sp_vz": "0",
                "latest_detector_capture_s": "1.900",
                "stop_reason": "none",
                "scenario": "proximity_stop",
            },
            {
                "event": "setpoint",
                "host_time_s": "2.100",
                "sp_vx": "0",
                "sp_vy": "0",
                "sp_vz": "0",
                "latest_detector_capture_s": "1.900",
                "stop_reason": "proximity_emergency",
                "scenario": "proximity_stop",
            },
        ]
    )
    result = ANALYZER.analyze(rows, [0.8])
    assert result["tracker_samples"] == 80
    assert result["detector_samples"] == 40
    assert result["tracker_only_samples"] == 40
    assert result["detector_stride_frames"] == 2
    assert abs(result["tracker_fps"] - 20.0) < 1e-6
    assert abs(result["detector_fps"] - 10.0) < 1e-6
    assert result["proximity_capture_to_zero_samples"] == 1
    assert abs(result["proximity_capture_to_zero_p99_s"] - 0.2) < 1e-9
    assert result["checks"]["crossing_identity_retained"]


def passing_timing(vmax):
    return {
        "gate_5_pass": True,
        "checks": {
            "crossing_identity_retained": True,
            "proximity_capture_to_zero_recorded": True,
        },
        "selected_vmax_mps": vmax,
        "reaction_time_s": 0.3,
        "target_freshness_s": 0.25,
        "stream_hz": 20,
        "ema_alpha": 0.7,
    }


def test_configuration_tie_prefers_10hz_only_after_all_gates_pass():
    selected = SELECTOR.select_configuration(
        passing_timing(1.0),
        passing_timing(1.0),
        {"calibration_gate_pass": True},
    )
    assert selected["selected"]["inference_hz"] == 10
    failed_10 = passing_timing(1.0)
    failed_10["checks"]["crossing_identity_retained"] = False
    selected = SELECTOR.select_configuration(
        passing_timing(1.0),
        failed_10,
        {"calibration_gate_pass": True},
    )
    assert selected["selected"]["inference_hz"] == 20
