# Completion and health marker for one competition-server traffic snapshot.
# Obstacles with the same sequence are committed only after every listed index
# has arrived.

std_msgs/Header header
uint64 sequence
bool connected
bool healthy
string reason
uint8[] aircraft_indices
