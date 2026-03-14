"""Tests for AMSL altitude fusion behavior across synthetic scenarios."""

from navisar.altitude_fusion.simulation import run_scenario


def _last(rows):
    return rows[-1]


def test_flat_takeoff_landing_tracks_amsl():
    rows = run_scenario("flat_takeoff_landing")
    end = _last(rows)
    assert abs(end["fused_amsl_m"] - end["true_amsl_m"]) < 0.8


def test_terrace_edge_drop_does_not_snap_to_ground():
    rows = run_scenario("terrace_edge_drop")
    post = [r for r in rows if r["t"] > 6.5]
    assert post
    mean_err = sum(abs(r["fused_amsl_m"] - r["true_amsl_m"]) for r in post) / len(post)
    assert mean_err < 1.2


def test_lidar_dropout_stays_stable_baro_only():
    rows = run_scenario("lidar_dropout")
    dropout = [r for r in rows if 4.0 <= r["t"] <= 8.0]
    assert dropout
    assert all(r["mode"] == "lidar_invalid_baro_only" for r in dropout[:10])


def test_large_tilt_rejected():
    rows = run_scenario("large_tilt_reject")
    high_tilt = [r for r in rows if 4.0 <= r["t"] <= 6.0]
    assert high_tilt
    assert any(not r["lidar_valid"] for r in high_tilt)
