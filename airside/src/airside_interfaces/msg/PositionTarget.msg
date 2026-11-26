# Where the engine wants the position controller to take the drone

Coordinate target
# The route through target, in order, with target at path[target_index]
Coordinate[] path
uint8 target_index

# Nonzero for a lap leg. Defaults keep non-lap clients unchanged.
uint32 leg_id
uint8 route_index
Coordinate[] route_vertices
