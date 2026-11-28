"""
Live map of which part of the survey area good photos actually cover.
"""

from __future__ import annotations

from gnc.src.orthomosaic import validate_polygon
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

# Where a photo edge runs along the boundary, float error leaves needle-thin
# slivers. Shapely calls them valid, but they turn self-intersecting once
# converted to GPS for the planner. Anything narrower than twice this is far
# below one pixel at survey GSDs, so it is removed from gaps.
_SLIVER_M = 1e-3


class CoverageMap:
    """
    Union of accepted photo footprints over the survey boundary.

    All polygons are in the survey ENU frame (metres east/north of the
    boundary anchor). A rejected photo never counts, even if its footprint
    arrives after the rejection: the stitcher and the photo stream are not
    ordered with respect to each other.
    """

    def __init__(self, boundary_enu: Polygon) -> None:
        validate_polygon(boundary_enu)
        self._boundary = boundary_enu
        self._footprints: dict[str, Polygon] = {}
        self._rejected: set[str] = set()
        self._covered: BaseGeometry | None = None  # cache, cleared on change

    @property
    def boundary(self) -> Polygon:
        return self._boundary

    @property
    def photo_count(self) -> int:
        """
        Photos received with a footprint, rejected ones included.
        """

        return len(self._footprints)

    @property
    def rejected_count(self) -> int:
        """
        Rejected photos that have a footprint.
        """

        return len(self._rejected & self._footprints.keys())

    def add_photo(self, image_id: str, footprint: Polygon) -> None:
        """
        Records a photo. A repeated ``image_id`` replaces the earlier one.
        """

        if not footprint.is_valid or footprint.is_empty:
            raise ValueError(f"footprint of {image_id!r} is not a valid polygon")
        self._footprints[image_id] = footprint
        self._covered = None

    def mark_rejected(self, image_id: str) -> None:
        """
        Excludes a photo from coverage, whether or not it has arrived yet.
        """

        if image_id not in self._rejected:
            self._rejected.add(image_id)
            self._covered = None

    def covered(self) -> BaseGeometry:
        """
        Part of the boundary covered by accepted photos.
        """

        if self._covered is None:
            accepted = [
                footprint
                for image_id, footprint in self._footprints.items()
                if image_id not in self._rejected
            ]
            self._covered = unary_union(accepted).intersection(self._boundary)
        return self._covered

    def covered_fraction(self) -> float:
        """
        Covered area over boundary area, in [0, 1].
        """

        return min(1.0, self.covered().area / self._boundary.area)

    def gaps(self, min_area_m2: float) -> list[Polygon]:
        """
        Uncovered parts of the boundary larger than ``min_area_m2``.

        Largest first, so the order is stable for the same photos. A gap may
        have holes where a photo covered an island inside it. Slivers under
        2 mm wide are dropped.
        """

        if min_area_m2 < 0.0:
            raise ValueError("min_area_m2 must be non-negative")
        uncovered = self._boundary.difference(self.covered())
        # Shrink then grow back with square corners: removes slivers and
        # leaves everything wider unchanged.
        opened = uncovered.buffer(-_SLIVER_M, join_style="mitre").buffer(
            _SLIVER_M, join_style="mitre"
        )
        parts = [
            part
            for part in getattr(opened, "geoms", [opened])
            if isinstance(part, Polygon) and part.area > min_area_m2
        ]
        return sorted(
            parts,
            key=lambda part: (-part.area, part.centroid.x, part.centroid.y),
        )
