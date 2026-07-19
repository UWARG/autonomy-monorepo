# Plane equation n dot x + offset = 0 in the ProcessedMap frame.
# The normal is assumed to be unit length and point out of the building.
string id
geometry_msgs/Vector3 normal
float64 offset
# Row-major covariance for [normal.x, normal.y, normal.z, offset].
float64[16] covariance
