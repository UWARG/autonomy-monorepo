import dataclasses
import math

import cv2
import numpy as np
import pytest

from src.image_stitcher.camera import Camera
from src.image_stitcher.message import ImageMessage
from src.image_stitcher.quality import QualityLimits, rejection_reason, sharpness
from src.image_stitcher.synthetic import lawn_mower_mission, make_ground, take_photo

LIMITS = QualityLimits()


@pytest.fixture(scope="module")
def good():
    ground = make_ground(width_px=1600, height_px=1200)
    rgb, _ = take_photo(ground, Camera.load(), 40.0, -30.0, 25.0, 20.0)
    return ImageMessage(43.0, -80.0, 25.0, rgb, roll_deg=1.0, pitch_deg=-2.0)


def scaled(message, gain):
    img = np.clip(message.img.astype(float) * gain, 0, 255).astype(np.uint8)
    return dataclasses.replace(message, img=img)


def motion_blurred(message, length_px, angle_deg=0.0):
    kernel = np.zeros((length_px, length_px), np.float32)
    kernel[length_px // 2, :] = 1
    centre = ((length_px - 1) / 2, (length_px - 1) / 2)
    rotation = cv2.getRotationMatrix2D(centre, angle_deg, 1.0)
    kernel = cv2.warpAffine(kernel, rotation, (length_px, length_px))
    kernel /= kernel.sum()
    return dataclasses.replace(message, img=cv2.filter2D(message.img, -1, kernel))


def test_good_photo_passes(good):
    assert rejection_reason(good, LIMITS) is None


@pytest.mark.parametrize("sigma", [1.0, 2.0, 4.0])
def test_out_of_focus_photo_is_rejected(good, sigma):
    blurred = dataclasses.replace(good, img=cv2.GaussianBlur(good.img, (0, 0), sigma))
    assert rejection_reason(blurred, LIMITS).startswith("blurry")


@pytest.mark.parametrize("length_px", [5, 7, 15])
@pytest.mark.parametrize("angle_deg", [0.0, 45.0, 90.0])
def test_motion_blurred_photo_is_rejected(good, length_px, angle_deg):
    shaky = motion_blurred(good, length_px, angle_deg)
    assert rejection_reason(shaky, LIMITS).startswith("blurry")


@pytest.mark.parametrize("seed", [3, 4])
def test_whole_mission_sharp_passes_and_slight_blur_fails(seed):
    # Every photo of a full mission, so ground with little fine detail is
    # covered too (seed 3 includes a photo the first version of this check
    # wrongly called blurry).
    photos = lawn_mower_mission(
        make_ground(seed=seed),
        Camera.load(),
        altitude_m=25.0,
        altitude_noise_m=0.3,
        heading_noise_deg=3.0,
        drone_count=3,
        seed=seed,
    )
    for photo in photos:
        message = photo.message
        assert rejection_reason(message, LIMITS) is None, message.image_id
        soft = cv2.GaussianBlur(message.img, (0, 0), 0.7)
        assert sharpness(soft) < LIMITS.min_sharpness, message.image_id
        shaky = motion_blurred(message, 5)
        assert sharpness(shaky.img) < LIMITS.min_sharpness, message.image_id


def test_featureless_photo_is_rejected(good):
    grey = dataclasses.replace(good, img=np.full_like(good.img, 128))
    assert sharpness(grey.img) == 0.0
    assert rejection_reason(grey, LIMITS).startswith("blurry")


@pytest.mark.parametrize("gain", [2.0, 3.0])
def test_overexposed_photo_is_rejected(good, gain):
    assert rejection_reason(scaled(good, gain), LIMITS).startswith("overexposed")


def test_underexposed_photo_is_rejected(good):
    assert rejection_reason(scaled(good, 0.1), LIMITS).startswith("underexposed")


def test_dim_but_sharp_photo_passes(good):
    assert rejection_reason(scaled(good, 0.4), LIMITS) is None


@pytest.mark.parametrize(("roll", "pitch"), [(6.0, 0.0), (0.0, -6.0), (-20.0, 15.0)])
def test_tilted_photo_is_rejected(good, roll, pitch):
    tilted = dataclasses.replace(good, roll_deg=roll, pitch_deg=pitch)
    assert rejection_reason(tilted, LIMITS).startswith("tilted")


def test_tilt_limit_is_configurable(good):
    tilted = dataclasses.replace(good, roll_deg=6.0)
    assert rejection_reason(tilted, QualityLimits(max_tilt_deg=10.0)) is None


@pytest.mark.parametrize(
    ("roll", "pitch"), [(math.nan, 0.0), (0.0, math.nan), (None, 0.0), (0.0, None)]
)
def test_unknown_attitude_is_rejected(good, roll, pitch):
    unknown = dataclasses.replace(good, roll_deg=roll, pitch_deg=pitch)
    assert rejection_reason(unknown, LIMITS) == "no roll/pitch"
