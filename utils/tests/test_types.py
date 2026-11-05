import math

import pytest

from src.types import ImageFrame, Quaternion, TargetPosition


class TestImageFrame:
    def test_to_tuple_round_trips(self):
        assert ImageFrame(u=12.5, v=-3.0).to_tuple() == (12.5, -3.0)

    def test_validate_accepts_finite_pixels(self):
        ImageFrame(u=0.0, v=0.0).validate()

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    @pytest.mark.parametrize("axis", ["u", "v"])
    def test_validate_rejects_non_finite_pixels(self, axis, bad):
        frame = ImageFrame(**{axis: bad, "u" if axis == "v" else "v": 1.0})
        with pytest.raises(ValueError, match="must be finite"):
            frame.validate()


class TestTargetPosition:
    def test_to_array_is_forward_right_down(self):
        position = TargetPosition(forward_m=1.0, right_m=2.0, down_m=3.0)
        assert position.to_array() == [1.0, 2.0, 3.0]

    def test_range_is_the_euclidean_norm(self):
        position = TargetPosition(forward_m=3.0, right_m=0.0, down_m=4.0)
        assert position.range_m == pytest.approx(5.0)

    def test_range_of_the_origin_is_zero(self):
        assert TargetPosition(0.0, 0.0, 0.0).range_m == pytest.approx(0.0)


class TestQuaternionNormalized:
    def test_scales_to_unit_length(self):
        unit = Quaternion(w=0.0, x=0.0, y=0.0, z=7.0).normalized()
        assert unit.to_array() == pytest.approx([0.0, 0.0, 0.0, 1.0])

    def test_preserves_an_already_unit_quaternion(self):
        original = Quaternion(w=0.5, x=0.5, y=0.5, z=0.5)
        assert original.normalized().to_array() == pytest.approx(original.to_array())

    def test_result_always_has_unit_norm(self):
        unit = Quaternion(w=1.0, x=-2.0, y=3.0, z=-4.0).normalized()
        assert math.sqrt(sum(c * c for c in unit.to_array())) == pytest.approx(1.0)

    def test_does_not_mutate_the_original(self):
        original = Quaternion(w=0.0, x=0.0, y=0.0, z=7.0)
        original.normalized()
        assert original.to_array() == [0.0, 0.0, 0.0, 7.0]

    def test_rejects_a_zero_quaternion(self):
        with pytest.raises(ValueError, match="non-zero norm"):
            Quaternion(w=0.0, x=0.0, y=0.0, z=0.0).normalized()

    @pytest.mark.parametrize("bad", [float("nan"), float("inf")])
    def test_rejects_non_finite_components(self, bad):
        with pytest.raises(ValueError, match="must be finite"):
            Quaternion(w=bad, x=0.0, y=0.0, z=1.0).normalized()
