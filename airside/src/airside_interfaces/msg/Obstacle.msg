# Another aircraft the drone must keep away from

uint8 aircraft_index            # 0-9, stable per aircraft
float32 horizontal_keep_away    # Minimum safe horizontal separation in meters
Coordinate position
float32 speed                   # Meters per second
float32 direction               # Degrees, 0-360 clockwise from true north; heading along its current path leg
