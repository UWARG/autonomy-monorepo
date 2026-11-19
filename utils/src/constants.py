"""Global constants."""

import math

TWO_PI = 2.0 * math.pi
EARTH_RADIUS_M = 6371008.8  # IUGG mean Earth radius

# Below this, a vector is too short for its direction to mean anything.
MIN_VECTOR_NORM = 1e-9
