import py_trees
from engine.behaviors.camera.take_photo import TakePhoto
from engine.constants import PHOTO_CAPTURE_MAX_ATTEMPTS
from engine.subtrees.setpoints import create_setpoints_subtree


def _find(node, name):
    if node.name == name:
        return node
    for child in node.children:
        found = _find(child, name)
        if found is not None:
            return found
    return None


class _AlwaysFails(py_trees.behaviour.Behaviour):
    """Stand-in for TakePhoto that counts how often it gets a chance."""

    def __init__(self, name):
        super().__init__(name=name)
        self.attempts = 0

    def update(self):
        self.attempts += 1
        return py_trees.common.Status.FAILURE


def test_take_photo_is_wrapped_in_bounded_retry():
    tree = create_setpoints_subtree()
    retry = _find(tree, "RetryPhoto")
    assert retry is not None
    assert isinstance(retry, py_trees.decorators.Retry)
    assert isinstance(retry.decorated, TakePhoto)
    assert retry.num_failures == PHOTO_CAPTURE_MAX_ATTEMPTS


def test_rejected_photo_retries_at_the_waypoint_then_gives_up():
    # Replicates the shipped shape (Repeat > FailureIsSuccess > Sequence >
    # Retry > behaviour) with an always-failing child: bounded attempts per
    # waypoint, then the Repeat restarts instead of stalling forever.
    photo = _AlwaysFails("FakePhoto")
    serve = py_trees.composites.Sequence(
        name="ServeSetpoint",
        memory=True,
        children=[
            py_trees.decorators.Retry(
                name="RetryPhoto",
                child=photo,
                num_failures=PHOTO_CAPTURE_MAX_ATTEMPTS,
            )
        ],
    )
    tree = py_trees.trees.BehaviourTree(
        py_trees.decorators.Repeat(
            name="ServeSetpoints",
            child=py_trees.decorators.FailureIsSuccess(
                name="SkipFailedSetpoint", child=serve
            ),
            num_success=-1,
        )
    )
    tree.setup()
    # py_trees 2.x (Humble) drives the tree via tick_tock; with a 0 ms
    # period this is just a bounded number of ticks.
    tree.tick_tock(period_ms=0, number_of_iterations=60)

    # Two waypoints' worth of attempts happened, in bounded groups, with the
    # sequence restarting in between (no infinite stall on a dead camera).
    assert photo.attempts >= 2 * PHOTO_CAPTURE_MAX_ATTEMPTS
