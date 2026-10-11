"""
Bookkeeping behaviors that maintain the landing-pad and item-count blackboard
entries for the dropping sequence.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from engine import blackboard_keys
from engine.constants import INITIAL_ITEM_COUNT
from engine.ground_log import send_to_ground
from utils.src.waypoint_utils import parse_landing_pads_file

_LANDING_PADS_FILE_PARAMETER = "landing_pads_file"


class LoadLandingPads(py_trees.behaviour.Behaviour):
    """
    Loads the landing pads from a YAML config file onto the blackboard and
    initialises the payload item count.

    Parses the landing-pads file and writes the pad list to
    ``free_landing_pads``, then sets ``items_remaining`` to
    ``INITIAL_ITEM_COUNT``. Returns SUCCESS once loaded, FAILURE if the file is
    missing, malformed, or has no landing pads.
    """

    def __init__(self, name: str = "LoadLandingPads") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.FREE_LANDING_PADS, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=blackboard_keys.ITEMS_REMAINING, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

        default_path = (
            f"{get_package_share_directory('engine')}/config/landing_pads.yaml"
        )
        if not self._node.has_parameter(_LANDING_PADS_FILE_PARAMETER):
            self._node.declare_parameter(_LANDING_PADS_FILE_PARAMETER, default_path)

    def update(self) -> py_trees.common.Status:
        landing_pads_file = (
            self._node.get_parameter(_LANDING_PADS_FILE_PARAMETER)
            .get_parameter_value()
            .string_value
        )

        try:
            pads = parse_landing_pads_file(landing_pads_file)
        except (OSError, ValueError) as error:
            self._node.get_logger().error(
                f"{self.name}: failed to load '{landing_pads_file}': {error}"
            )
            return py_trees.common.Status.FAILURE

        if not pads:
            self._node.get_logger().error(
                f"{self.name}: no landing pads found in '{landing_pads_file}'"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.FREE_LANDING_PADS, list(pads))
        self.blackboard.set(blackboard_keys.ITEMS_REMAINING, INITIAL_ITEM_COUNT)
        self._node.get_logger().info(
            f"{self.name}: loaded {len(pads)} landing pads from "
            f"'{landing_pads_file}', {INITIAL_ITEM_COUNT} items on board: {pads}"
        )
        send_to_ground(
            self._node, f"ENG: dropping phase, {INITIAL_ITEM_COUNT} items on board"
        )
        return py_trees.common.Status.SUCCESS


class HoldingItem(py_trees.behaviour.Behaviour):
    """
    Succeeds while at least one payload item remains on board.

    Reads ``items_remaining`` and returns SUCCESS if it is greater than zero,
    FAILURE otherwise. A missing key is treated as no items remaining. This is
    the loop guard for the dropping sequence ("while holding at least one
    item").
    """

    def __init__(self, name: str = "HoldingItem") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.ITEMS_REMAINING, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            items_remaining = self.blackboard.get(blackboard_keys.ITEMS_REMAINING)
        except KeyError:
            self._node.get_logger().warning(
                f"{self.name}: no item count on the blackboard"
            )
            return py_trees.common.Status.FAILURE

        if items_remaining > 0:
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: no items remaining, dropping complete"
        )
        send_to_ground(self._node, "ENG: all items dropped")
        return py_trees.common.Status.FAILURE


class FindUnoccupiedLandingPad(py_trees.behaviour.Behaviour):
    """
    Claims the next unoccupied landing pad for a drop.

    Removes a pad from ``free_landing_pads`` and writes it to
    ``target_landing_pad``; removing it marks the pad occupied so a later drop
    cannot reselect it. Returns SUCCESS when a pad was claimed, FAILURE if no
    unoccupied pads remain.
    """

    def __init__(self, name: str = "FindUnoccupiedLandingPad") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.FREE_LANDING_PADS, access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key=blackboard_keys.TARGET_LANDING_PAD, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

    def update(self) -> py_trees.common.Status:
        try:
            free_pads = self.blackboard.get(blackboard_keys.FREE_LANDING_PADS)
        except KeyError:
            self._node.get_logger().error(
                f"{self.name}: no landing pads on the blackboard"
            )
            return py_trees.common.Status.FAILURE

        if not free_pads:
            self._node.get_logger().warning(
                f"{self.name}: no unoccupied landing pads left"
            )
            send_to_ground(self._node, "ENG: no unoccupied landing pads left")
            return py_trees.common.Status.FAILURE

        pad = free_pads[0]
        remaining_pads = free_pads[1:]
        self.blackboard.set(blackboard_keys.FREE_LANDING_PADS, remaining_pads)
        self.blackboard.set(blackboard_keys.TARGET_LANDING_PAD, pad)
        self._node.get_logger().info(
            f"{self.name}: claimed pad {pad}, {len(remaining_pads)} pads still free"
        )
        return py_trees.common.Status.SUCCESS
