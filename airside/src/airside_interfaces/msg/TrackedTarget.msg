# Capture-stamped, camera-frame position of one explicitly owned track.
std_msgs/Header header
# Host receipt from the DepthAI queue and ROS publication, for HITL timing.
builtin_interfaces/Time host_receipt_stamp
builtin_interfaces/Time publish_stamp
geometry_msgs/Point position
int64 track_id
uint64 sequence_num
