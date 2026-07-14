"""Gate 4 (offline) -- Monte-Carlo soak of the deployed configuration.

The unmarked smoke test (25 episodes) runs in every pytest invocation and keeps
the machinery honest. The full 500-episode soak is behind the ``soak`` marker
(deselected by default -- run ``pytest -m soak`` or the CLI, which also writes
CSV + histogram artifacts to gnc/sim_output/).
"""

import pytest

from sim.monte_carlo import make_episode, run_episode, soak


def test_episodes_are_deterministic_per_seed():
    assert make_episode(1234) == make_episode(1234)
    _, r1, m1 = run_episode(1234)
    _, r2, m2 = run_episode(1234)
    assert m1 == m2
    assert r1.true_range == r2.true_range


def test_episode_metrics_shape():
    _, _, metrics = run_episode(7)
    for key in (
        "seed",
        "min_range",
        "unsafe",
        "max_fwd_cmd",
        "fwd_cmd_bounded",
        "convergible",
        "relatch_count",
        "action_toggles",
    ):
        assert key in metrics


def test_smoke_soak_25_episodes(tmp_path):
    ok, summary, rows = soak(25, base_seed=42, outdir=str(tmp_path))
    assert summary["episodes"] == 25 and len(rows) == 25
    assert summary["unsafe"] == 0, f"unsafe seeds: {summary['unsafe_seeds']}"
    assert summary["fwd_cmd_unbounded"] == 0
    assert ok, summary


@pytest.mark.soak
def test_full_soak_500_episodes(tmp_path):
    ok, summary, _ = soak(500, base_seed=42, outdir=str(tmp_path))
    assert summary["unsafe"] == 0, f"unsafe seeds: {summary['unsafe_seeds']}"
    assert ok, summary
