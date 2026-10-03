import pytest
from shapely.geometry import Polygon
from shapely.ops import unary_union

from orthomosaic.decompose import (
    HORIZONTAL,
    VERTICAL,
    Cell,
    _decompose,
    _slice_count,
    decompose_polygon,
    is_monotone,
    order_cells,
    validate_polygon,
)

RECT = [(0, 0), (10, 0), (10, 10), (0, 10)]
U_SHAPE = [(0, 0), (10, 0), (10, 10), (7, 10), (7, 5), (3, 5), (3, 10), (0, 10)]
C_SHAPE = [(0, 0), (10, 0), (10, 3), (5, 3), (5, 7), (10, 7), (10, 10), (0, 10)]
H_SHAPE = [
    (0, 0),
    (3, 0),
    (3, 4),
    (7, 4),
    (7, 0),
    (10, 0),
    (10, 10),
    (7, 10),
    (7, 6),
    (3, 6),
    (3, 10),
    (0, 10),
]
L_SHAPE = [(0, 0), (10, 0), (10, 10), (6, 10), (6, 6), (0, 6)]

ALL_SHAPES = {
    "rect": RECT,
    "u": U_SHAPE,
    "c": C_SHAPE,
    "h": H_SHAPE,
    "l": L_SHAPE,
}


def cell_polygons(polygon: Polygon):
    return [cell.polygon for cell in decompose_polygon(polygon, HORIZONTAL)]


@pytest.mark.parametrize("coords", [RECT, U_SHAPE, C_SHAPE, H_SHAPE, L_SHAPE])
def test_shape_is_valid_simple_polygon(coords):
    polygon = Polygon(coords)
    assert polygon.is_valid
    assert not polygon.interiors


@pytest.mark.parametrize(
    "name,expected_count",
    [
        ("rect", 1),
        ("u", 3),
        ("c", 1),
        ("h", 5),
        ("l", 1),
    ],
)
def test_cell_counts(name, expected_count):
    polygon = Polygon(ALL_SHAPES[name])
    assert len(cell_polygons(polygon)) == expected_count


@pytest.mark.parametrize("coords", [RECT, U_SHAPE, C_SHAPE, H_SHAPE, L_SHAPE])
def test_union_recovers_polygon(coords):
    polygon = Polygon(coords)
    cells = cell_polygons(polygon)
    union = unary_union(cells)
    tolerance = 1e-6
    assert polygon.symmetric_difference(union).area < tolerance
    assert abs(union.area - polygon.area) < tolerance


@pytest.mark.parametrize("coords", [RECT, U_SHAPE, C_SHAPE, H_SHAPE, L_SHAPE])
def test_no_gaps_or_overlaps_between_cells(coords):
    polygon = Polygon(coords)
    cells = cell_polygons(polygon)
    total_area = sum(cell.area for cell in cells)
    assert abs(total_area - polygon.area) < 1e-6


@pytest.mark.parametrize("coords", [RECT, U_SHAPE, C_SHAPE, H_SHAPE, L_SHAPE])
def test_each_cell_inside_original_polygon(coords):
    polygon = Polygon(coords)
    cells = cell_polygons(polygon)
    for cell in cells:
        assert cell.difference(polygon).area < 1e-6


@pytest.mark.parametrize("coords", [RECT, U_SHAPE, C_SHAPE, H_SHAPE, L_SHAPE])
def test_each_cell_is_monotone_along_its_axis(coords):
    polygon = Polygon(coords)
    for cell in decompose_polygon(polygon, HORIZONTAL):
        assert is_monotone(cell.polygon, cell.axis)


def test_decomposition_is_orientation_independent():
    reversed_u = list(reversed(U_SHAPE))
    assert len(cell_polygons(Polygon(reversed_u))) == 3


def test_u_shape_is_not_horizontally_monotone():
    assert not is_monotone(Polygon(U_SHAPE), HORIZONTAL)


def test_rectangle_is_horizontally_monotone():
    assert is_monotone(Polygon(RECT), HORIZONTAL)


def test_slice_count_matches_expectations():
    polygon = Polygon(U_SHAPE)
    assert _slice_count(polygon, HORIZONTAL, 2.0) == 1
    assert _slice_count(polygon, HORIZONTAL, 6.0) == 2


def test_decompose_vertical_axis_rectangle():
    cells = decompose_polygon(Polygon(RECT), VERTICAL)
    assert len(cells) == 1
    assert cells[0].axis == VERTICAL


def test_validate_polygon_rejects_holes():
    ring = Polygon(
        shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(3, 3), (7, 3), (7, 7), (3, 7)]],
    )
    with pytest.raises(ValueError, match="holes"):
        validate_polygon(ring)


def test_validate_polygon_rejects_self_intersecting():
    bowtie = Polygon([(0, 0), (10, 10), (0, 10), (10, 0)])
    with pytest.raises(ValueError, match="self-intersecting"):
        validate_polygon(bowtie)


def test_validate_polygon_rejects_empty():
    with pytest.raises(ValueError, match="empty"):
        validate_polygon(Polygon())


def test_validate_polygon_rejects_degenerate():
    degenerate = Polygon([(0, 0), (1, 0), (2, 0)])
    with pytest.raises(ValueError):
        validate_polygon(degenerate)


def test_recursion_depth_cap_raises_on_hole():
    ring = Polygon(
        shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(3, 3), (7, 3), (7, 7), (3, 7)]],
    )
    with pytest.raises(ValueError, match="did not converge"):
        _decompose(ring, HORIZONTAL, depth=0)


def test_decompose_polygon_rejects_holes_via_validation():
    ring = Polygon(
        shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(3, 3), (7, 3), (7, 7), (3, 7)]],
    )
    with pytest.raises(ValueError, match="holes"):
        decompose_polygon(ring)


def test_band_intersection_with_rotated_rectangle():
    polygon = Polygon([(0, 0), (10, 0), (12, 8), (2, 8)])
    cells = cell_polygons(polygon)
    union = unary_union(cells)
    assert union.symmetric_difference(polygon).area < 1e-6
    for cell in cells:
        assert is_monotone(cell, HORIZONTAL)


def test_double_notch_decomposes_into_5_cells():
    double_notch = [
        (0, 0),
        (4, 0),
        (4, 3),
        (6, 3),
        (6, 0),
        (10, 0),
        (10, 10),
        (8, 10),
        (8, 6),
        (2, 6),
        (2, 10),
        (0, 10),
    ]
    polygon = Polygon(double_notch)
    cells = cell_polygons(polygon)
    assert len(cells) == 5
    union = unary_union(cells)
    assert polygon.symmetric_difference(union).area < 1e-6
    assert abs(sum(cell.area for cell in cells) - polygon.area) < 1e-6
    for cell in cells:
        assert is_monotone(cell, HORIZONTAL)


def _rect_cell(xmin, xmax, ymin, ymax):
    return Cell(Polygon([(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]), HORIZONTAL)


def test_order_cells_prefers_nearest_to_reference():
    left = _rect_cell(0, 10, 0, 10)
    middle = _rect_cell(20, 30, 0, 10)
    right = _rect_cell(40, 50, 0, 10)
    cells = [left, right, middle]

    ordered = order_cells(cells, reference_point=(25.0, 5.0))
    assert ordered[0] is middle
    assert set(ordered) == set(cells)

    ordered_far = order_cells(cells, reference_point=(45.0, 5.0))
    assert ordered_far[0] is right


def test_order_cells_default_reference_is_origin():
    near = _rect_cell(-1, 1, -1, 1)
    far = _rect_cell(50, 60, 50, 60)
    ordered = order_cells([far, near])
    assert ordered[0] is near


def test_order_cells_empty():
    assert order_cells([]) == []


def test_order_cells_is_continuous_nearest_neighbour():
    a = _rect_cell(0, 10, 0, 10)
    b = _rect_cell(10.5, 20.5, 0, 10)
    c = _rect_cell(40, 50, 0, 10)
    ordered = order_cells([a, b, c], reference_point=(0.0, 5.0))
    assert ordered == [a, b, c]