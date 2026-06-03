

"""
Shared sensor UDP ports for integration tests.
"""

HOST = "127.0.0.1"
GROUNDSIDE_OFFSET = 1

CAMERA_PORTS = {
    6000:{
        "port": 6000,
        "direction": [0,0,-1],
        "fov": 60,
        "near": 0.1,
        "far": 100.0,
        "height": 224,
        "width": 224
    },
    6002:{
        "port": 6002,
        "direction": [1,0,0],
        "fov": 60,
        "near": 0.1,
        "far": 100.0,
        "height": 224,
        "width": 224
    }
}
RANGE_FINDER_PORTS = {
    6004:{
        "port": 6004,
        "direction": [0,0,-1],
        "dist": 100
    }
}
