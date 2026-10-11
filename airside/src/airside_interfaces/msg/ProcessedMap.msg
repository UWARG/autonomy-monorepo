# Complete post-processing snapshot for building-relative target localization.
# header.frame_id is assumed to be mission_frd: +x north, +y east, +z down.
std_msgs/Header header
Plane[] planes
string ground_plane_id
float64 building_height_m
float64 building_height_stddev_m
BuildingWing[] wings
SpatialTarget[] targets
