"""
Detection - Shared output type for any camera that runs object detection.

This is the camera-agnostic contract. Both the OAK-D (on-device inference) and
the ArduCam (host-side inference, later) must return a list[Detection], so that
downstream consumers don't care which camera produced them.

TODO(you): decide and lock the conventions below, since ArduCam will have to match:
  - bbox format: corner coords (x1, y1, x2, y2) vs. (x, y, w, h)?
  - coordinate space: pixels in the RGB frame (recommended) vs. normalized 0-1?
  - what does `depth` mean when there is no depth (e.g. ArduCam without mapping)?
"""

from dataclasses import dataclass
import math


@dataclass
class Detection:
    """A single detected target."""

    # --- bounding box, in PIXELS of the RGB frame ---
    # DepthAI gives you normalized coords (0.0-1.0). You convert using the
    # frame WIDTH/HEIGHT before constructing this. Pick ONE convention and
    # document it here.
    x1: float
    y1: float
    x2: float
    y2: float

    # --- classification ---
    confidence: float
    label: int  # class id from the model

    # --- depth (metres) ---
    # On OAK-D with a SpatialDetectionNetwork this comes back fused with the
    # bbox already. On ArduCam it may be NaN unless depth mapping is configured.
    depth: float = math.nan

    @property
    def center(self) -> tuple[float, float]:
        """Pixel center (u, v) of the bbox. Handy for target localization."""
        # TODO(you): return the midpoint of the box.
        raise NotImplementedError
