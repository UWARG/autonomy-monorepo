# Capture-stamped, camera-frame position of one explicitly owned track.
std_msgs/Header header
# Host receipt from the DepthAI queue and ROS publication, for HITL timing.
builtin_interfaces/Time host_receipt_stamp
builtin_interfaces/Time publish_stamp
geometry_msgs/Point position
int64 track_id
uint64 sequence_num
# Latest detector-confirmed spatial sample. Tracker-only frames carry this
# unchanged and must not refresh safety or control age.
builtin_interfaces/Time detector_stamp
uint64 detector_sequence_num
bool detector_confirmed
# False means the owned target is outside the 3.0 m validated envelope.
bool within_validated_range
