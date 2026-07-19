# Structured localization result for one target.
uint8 OK=0
uint8 INVALID_TARGET=1
uint8 MAP_INVALID=2
uint8 NO_SURFACE=3
uint8 AMBIGUOUS_SURFACE=4
uint8 AMBIGUOUS_ANCHOR=5
uint8 UNSTABLE_GEOMETRY=6

uint8 SURFACE_NONE=0
uint8 SURFACE_WALL=1
uint8 SURFACE_GROUND=2

uint8 ANCHOR_NONE=0
uint8 ANCHOR_OUTER_CORNER=1
uint8 ANCHOR_INNER_CORNER=2
uint8 ANCHOR_TWO_CORNERS=3

string target_id
string colour
uint8 status
string reason
geometry_msgs/Point source_position
geometry_msgs/Point snapped_position
string surface_id
uint8 surface_type
string anchor_id
uint8 anchor_type
ReferenceMeasurement[] measurements
float64 uncertainty_95_m
string description
