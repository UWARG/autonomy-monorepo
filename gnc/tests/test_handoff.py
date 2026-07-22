"""Gate 0 -- the SITL auto-handoff sequencer (pure)."""

from handoff import HandoffAction, HandoffConfig, HandoffSequencer

CFG = HandoffConfig(takeoff_alt_m=3.0, alt_fraction=0.95, climb_timeout_s=20.0, arm_retry_s=2.0)


def test_nominal_sequence():
    seq = HandoffSequencer(CFG)
    assert seq.step(False, "", False, 0.0, 0.0) is HandoffAction.WAIT
    assert seq.step(True, "STABILIZE", False, 0.0, 1.0) is HandoffAction.REQUEST_GUIDED
    assert seq.step(True, "GUIDED", False, 0.0, 2.0) is HandoffAction.REQUEST_ARM
    assert seq.step(True, "GUIDED", True, 0.0, 3.0) is HandoffAction.REQUEST_TAKEOFF
    seq.notify_takeoff_sent(3.0)
    assert seq.step(True, "GUIDED", True, 1.0, 4.0) is HandoffAction.WAIT  # climbing
    assert seq.step(True, "GUIDED", True, 2.9, 8.0) is HandoffAction.COMPLETE
    assert seq.is_complete


def test_arm_retry_backoff():
    seq = HandoffSequencer(CFG)
    assert seq.step(True, "GUIDED", False, 0.0, 0.0) is HandoffAction.REQUEST_ARM
    # within the retry window: wait, do not spam the arming service
    assert seq.step(True, "GUIDED", False, 0.0, 0.5) is HandoffAction.WAIT
    assert seq.step(True, "GUIDED", False, 0.0, 1.9) is HandoffAction.WAIT
    assert seq.step(True, "GUIDED", False, 0.0, 2.1) is HandoffAction.REQUEST_ARM


def test_takeoff_retries_until_sent():
    """An unready takeoff service must not strand the sequencer (no silent skip)."""
    seq = HandoffSequencer(CFG)
    assert seq.step(True, "GUIDED", True, 0.0, 0.0) is HandoffAction.REQUEST_TAKEOFF
    # executor could not send (service unready): throttled retry, then re-request
    assert seq.step(True, "GUIDED", True, 0.0, 0.1) is HandoffAction.WAIT
    assert seq.step(True, "GUIDED", True, 0.0, 2.5) is HandoffAction.REQUEST_TAKEOFF
    seq.notify_takeoff_sent(2.5)
    assert seq.step(True, "GUIDED", True, 0.0, 2.6) is HandoffAction.WAIT  # climbing now


def test_never_complete_low_inside_window():
    seq = HandoffSequencer(CFG)
    seq.step(True, "GUIDED", True, 0.0, 0.0)
    seq.notify_takeoff_sent(0.0)
    for t in (1.0, 5.0, 10.0, 19.0):
        assert seq.step(True, "GUIDED", True, 0.5, t) is HandoffAction.WAIT


def test_climb_timeout_retries_takeoff_never_completes_grounded():
    """A climb window that expires with no measured altitude means the takeoff
    was likely rejected -- COMPLETE here would declare a parked drone airborne
    (observed in SITL: full-speed follow commands at a target 3 m overhead).
    The sequencer must go back to requesting takeoff instead."""
    seq = HandoffSequencer(CFG)
    seq.step(True, "GUIDED", True, 0.0, 0.0)
    seq.notify_takeoff_sent(0.0)
    assert seq.step(True, "GUIDED", True, 0.0, 21.0) is HandoffAction.WAIT
    assert seq.step(True, "GUIDED", True, 0.0, 21.1) is HandoffAction.REQUEST_TAKEOFF
    assert not seq.is_complete
    # the retried takeoff works this time
    seq.notify_takeoff_sent(21.1)
    assert seq.step(True, "GUIDED", True, 2.9, 30.0) is HandoffAction.COMPLETE


def test_complete_is_sticky():
    seq = HandoffSequencer(CFG)
    seq.step(True, "GUIDED", True, 0.0, 0.0)
    seq.notify_takeoff_sent(0.0)
    seq.step(True, "GUIDED", True, 3.0, 5.0)  # COMPLETE
    # even if inputs later look pre-takeoff, the sequencer never re-runs
    assert seq.step(True, "STABILIZE", False, 0.0, 6.0) is HandoffAction.COMPLETE


def test_mark_complete_skips_bring_up():
    seq = HandoffSequencer(CFG)
    seq.mark_complete()
    assert seq.is_complete
    assert seq.step(True, "STABILIZE", False, 0.0, 0.0) is HandoffAction.COMPLETE
