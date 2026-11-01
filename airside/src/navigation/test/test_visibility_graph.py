from __future__ import annotations

import math

import pytest
from navigation.constants import (
    ESCAPE_DISTANCE_M,
    FORWARD_ZONE_LOOKAHEAD_S,
    FORWARD_ZONE_MAX_EXTENSION_M,
    KEEP_AWAY_MARGIN_M,
    ZONE_CORNER_COUNT,
    ZONE_CORNER_STANDOFF_M,
)
from navigation.visibility_graph import (
    KeepAwayZone,
    PlanStatus,
    Point,
    VisibilityGraphPlanner,
)


KEEP_AWAY_M = 20.0
ZONE_RADIUS_M = KEEP_AWAY_M + KEEP_AWAY_MARGIN_M


def _stationary_zone(position: Point) -> KeepAwayZone:
    return KeepAwayZone.from_obstacle(position, KEEP_AWAY_M, 0.0, 0.0)


def _fly(
    start: Point,
    destination: Point,
    zones: list[KeepAwayZone],
    step_m: float = 2.0,
    max_steps: int = 2000,
) -> list[Point]:
    """
    Flies a simulated drone that moves ``step_m`` towards each planned point,
    returning the track it flew. Stops on arrival or after ``max_steps``.
    """

    planner = VisibilityGraphPlanner()
    position = start
    track = [position]
    for _ in range(max_steps):
        if math.dist(position, destination) <= step_m:
            break

        point = planner.plan(position, destination, zones).point
        distance_m = math.dist(position, point)
        if distance_m < 1e-9:
            break

        fraction = min(1.0, step_m / distance_m)
        position = (
            position[0] + (point[0] - position[0]) * fraction,
            position[1] + (point[1] - position[1]) * fraction,
        )
        track.append(position)
    return track


def _track_length_m(track: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(track, track[1:]))


class TestKeepAwayZone:
    def test_stationary_zone_is_a_circle_with_margin(self) -> None:
        zone = _stationary_zone((0.0, 0.0))

        for point in [(ZONE_RADIUS_M, 0.0), (0.0, -ZONE_RADIUS_M)]:
            assert zone.point_clearance_m(point) == pytest.approx(0.0)

    def test_zone_reaches_further_ahead_than_behind(self) -> None:
        speed_mps = 10.0
        # Heading east
        zone = KeepAwayZone.from_obstacle((0.0, 0.0), KEEP_AWAY_M, speed_mps, 90.0)
        ahead_m = ZONE_RADIUS_M + speed_mps * FORWARD_ZONE_LOOKAHEAD_S

        assert zone.point_clearance_m((ahead_m, 0.0)) == pytest.approx(0.0)
        assert zone.point_clearance_m((-ZONE_RADIUS_M, 0.0)) == pytest.approx(0.0)
        assert zone.point_clearance_m((0.0, ZONE_RADIUS_M)) == pytest.approx(0.0)

    def test_forward_extension_is_capped(self) -> None:
        zone = KeepAwayZone.from_obstacle((0.0, 0.0), KEEP_AWAY_M, 1000.0, 0.0)

        assert math.dist(zone.tail, zone.nose) == pytest.approx(
            FORWARD_ZONE_MAX_EXTENSION_M
        )

    def test_path_through_zone_has_negative_clearance(self) -> None:
        zone = _stationary_zone((0.0, 0.0))

        assert zone.segment_clearance_m((-100.0, 0.0), (100.0, 0.0)) == pytest.approx(
            -ZONE_RADIUS_M
        )
        assert zone.segment_clearance_m((-100.0, 40.0), (100.0, 40.0)) == pytest.approx(
            40.0 - ZONE_RADIUS_M
        )

    def test_blocks_matches_clearance(self) -> None:
        zone = KeepAwayZone.from_obstacle((0.0, 0.0), KEEP_AWAY_M, 10.0, 90.0)

        assert zone.blocks((30.0, -100.0), (30.0, 100.0))
        assert not zone.blocks((-30.0, -100.0), (-30.0, 100.0))
        assert not zone.blocks((500.0, -100.0), (500.0, 100.0))

    @pytest.mark.parametrize("speed_mps", [0.0, 10.0])
    def test_corners_ring_the_zone_just_outside_it(self, speed_mps: float) -> None:
        zone = KeepAwayZone.from_obstacle((5.0, -3.0), KEEP_AWAY_M, speed_mps, 30.0)

        corners = zone.corners()

        assert len(corners) == ZONE_CORNER_COUNT
        # Every side of the ring stays the standoff clear of the zone
        for start, end in zip(corners, corners[1:] + corners[:1]):
            assert zone.segment_clearance_m(start, end) == pytest.approx(
                ZONE_CORNER_STANDOFF_M
            )


class TestVisibilityGraphPlanner:
    def test_no_obstacles_goes_direct(self) -> None:
        plan = VisibilityGraphPlanner().plan((0.0, 0.0), (0.0, 200.0), [])

        assert plan.status == PlanStatus.DIRECT
        assert plan.point == (0.0, 200.0)

    def test_obstacle_off_the_path_goes_direct(self) -> None:
        zones = [_stationary_zone((100.0, 100.0))]

        plan = VisibilityGraphPlanner().plan((0.0, 0.0), (0.0, 200.0), zones)

        assert plan.status == PlanStatus.DIRECT
        assert plan.point == (0.0, 200.0)

    def test_detours_as_soon_as_the_path_is_blocked(self) -> None:
        # Obstacle a long way off, but on the path
        zones = [_stationary_zone((0.0, 400.0))]

        plan = VisibilityGraphPlanner().plan((0.0, 0.0), (0.0, 800.0), zones)

        assert plan.status == PlanStatus.DETOUR
        assert abs(plan.point[0]) > ZONE_RADIUS_M
        assert zones[0].segment_clearance_m((0.0, 0.0), plan.point) >= 0.0

    def test_detours_towards_the_open_side(self) -> None:
        # Obstacle slightly east of the path: shorter way round is to the west
        zones = [_stationary_zone((10.0, 100.0))]

        plan = VisibilityGraphPlanner().plan((0.0, 0.0), (0.0, 200.0), zones)

        assert plan.status == PlanStatus.DETOUR
        assert plan.point[0] < 0.0

    def test_keeps_its_side_when_obstacle_is_dead_ahead(self) -> None:
        zones = [_stationary_zone((0.0, 100.0))]
        planner = VisibilityGraphPlanner()

        first = planner.plan((0.0, 0.0), (0.0, 200.0), zones)
        # Nudged so that the other way round is now slightly shorter
        nudge_east_m = -0.5 if first.point[0] > 0.0 else 0.5
        second = planner.plan((nudge_east_m, 0.0), (0.0, 200.0), zones)

        assert (first.point[0] > 0.0) == (second.point[0] > 0.0)

    def test_takes_the_shorter_way_round_a_group(self) -> None:
        # A wall of overlapping zones reaching far to the east: going round the
        # first one to the east is a dead end, the way round is to the west
        zones = [_stationary_zone((east, 100.0)) for east in (10.0, 50.0, 90.0, 130.0)]
        destination = (0.0, 200.0)

        plan = VisibilityGraphPlanner().plan((0.0, 0.0), destination, zones)
        track = _fly((0.0, 0.0), destination, zones)

        assert plan.point[0] < 0.0
        assert math.dist(track[-1], destination) <= 2.0
        assert all(point[0] < 1.0 for point in track)

    def test_moves_on_from_a_corner_it_is_standing_on(self) -> None:
        zones = [_stationary_zone((0.0, 100.0))]
        corner = min(zones[0].corners(), key=lambda point: point[1])

        plan = VisibilityGraphPlanner().plan(corner, (0.0, 200.0), zones)

        assert math.dist(corner, plan.point) > 1.0

    def test_inside_zone_escapes_straight_out(self) -> None:
        zones = [_stationary_zone((0.0, 0.0))]

        plan = VisibilityGraphPlanner().plan((5.0, 0.0), (0.0, 200.0), zones)

        assert plan.status == PlanStatus.ESCAPE
        assert plan.point == pytest.approx((5.0 + ESCAPE_DISTANCE_M, 0.0))

    def test_inside_forward_zone_escapes_sideways(self) -> None:
        # Obstacle heading north, drone just east of its path ahead of it
        zones = [KeepAwayZone.from_obstacle((0.0, 0.0), KEEP_AWAY_M, 10.0, 0.0)]

        plan = VisibilityGraphPlanner().plan((5.0, 30.0), (0.0, 200.0), zones)

        assert plan.status == PlanStatus.ESCAPE
        assert plan.point == pytest.approx((5.0 + ESCAPE_DISTANCE_M, 30.0))

    def test_destination_inside_zone_waits_at_its_edge(self) -> None:
        zones = [_stationary_zone((0.0, 200.0))]
        destination = (0.0, 190.0)

        track = _fly((0.0, 0.0), destination, zones)

        assert all(zones[0].point_clearance_m(point) >= -1e-6 for point in track)
        assert zones[0].point_clearance_m(track[-1]) == pytest.approx(
            ZONE_CORNER_STANDOFF_M, abs=2.0
        )
        assert track[-1][1] < 200.0

    def test_walled_in_holds_position(self) -> None:
        # Ring of zones around the drone with no gap between them
        zones = [
            _stationary_zone(
                (
                    80.0 * math.sin(math.radians(bearing)),
                    80.0 * math.cos(math.radians(bearing)),
                )
            )
            for bearing in range(0, 360, 30)
        ]

        plan = VisibilityGraphPlanner().plan((0.0, 0.0), (0.0, 300.0), zones)

        assert plan.status == PlanStatus.HOLD
        assert plan.point == (0.0, 0.0)

    def test_flies_around_stationary_obstacle(self) -> None:
        zones = [_stationary_zone((0.0, 100.0))]
        destination = (0.0, 200.0)

        track = _fly((0.0, 0.0), destination, zones)

        assert math.dist(track[-1], destination) <= 2.0
        assert all(zones[0].point_clearance_m(point) >= -1e-6 for point in track)
        # Close to the shortest way there is: two tangents and an arc
        tangent_m = math.sqrt(100.0**2 - ZONE_RADIUS_M**2)
        arc_m = 2.0 * ZONE_RADIUS_M * math.asin(ZONE_RADIUS_M / 100.0)
        assert _track_length_m(track) < 1.05 * (2.0 * tangent_m + arc_m)

    def test_flies_around_several_obstacles(self) -> None:
        zones = [
            _stationary_zone((0.0, 80.0)),
            _stationary_zone((45.0, 150.0)),
            # Heading west across the path
            KeepAwayZone.from_obstacle((60.0, 230.0), KEEP_AWAY_M, 10.0, 270.0),
        ]
        destination = (0.0, 320.0)

        track = _fly((0.0, 0.0), destination, zones)

        assert math.dist(track[-1], destination) <= 2.0
        for zone in zones:
            assert all(zone.point_clearance_m(point) >= -1e-6 for point in track)

    def test_passes_behind_rather_than_through_forward_zone(self) -> None:
        # Obstacle east of the path heading west across it: its forward zone
        # blocks the direct way, although its plain circle would not
        zone = KeepAwayZone.from_obstacle((40.0, 100.0), KEEP_AWAY_M, 10.0, 270.0)
        destination = (0.0, 200.0)

        track = _fly((0.0, 0.0), destination, [zone])

        assert _stationary_zone((40.0, 100.0)).segment_clearance_m(
            (0.0, 0.0), destination
        ) > 0.0
        assert math.dist(track[-1], destination) <= 2.0
        assert all(zone.point_clearance_m(point) >= -1e-6 for point in track)
        assert max(abs(point[0]) for point in track) > 1.0

    def test_stays_clear_of_obstacle_crossing_its_path(self) -> None:
        # Obstacle reported once a second, frozen in between, crossing the
        # drone's path at the moment the drone would get there
        speed_mps = 8.0
        destination = (0.0, 400.0)
        planner = VisibilityGraphPlanner()
        position = (0.0, 0.0)
        closest_m = math.inf

        for tick in range(400):
            time_s = tick * 0.5
            if math.dist(position, destination) <= 4.0:
                break

            obstacle_now = (200.0 - speed_mps * time_s, 200.0)
            reported_s = math.floor(time_s)
            reported = (200.0 - speed_mps * reported_s, 200.0)
            zone = KeepAwayZone.from_obstacle(reported, KEEP_AWAY_M, speed_mps, 270.0)

            point = planner.plan(position, destination, [zone]).point
            distance_m = math.dist(position, point)
            if distance_m > 1e-9:
                fraction = min(1.0, 4.0 / distance_m)
                position = (
                    position[0] + (point[0] - position[0]) * fraction,
                    position[1] + (point[1] - position[1]) * fraction,
                )
            closest_m = min(closest_m, math.dist(position, obstacle_now))

        assert math.dist(position, destination) <= 4.0
        assert closest_m >= KEEP_AWAY_M
