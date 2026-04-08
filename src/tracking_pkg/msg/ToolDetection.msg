# Single detected tool on the instrument tray
# Published by the tray perception node

std_msgs/Header header

string tool_id                              # Persistent tracking ID (e.g. "tool_3")
string tool_class                           # Class name from OBB model (e.g. "needle_holder")
float32 confidence                          # Detection confidence [0.0, 1.0]

tracking_pkg/OrientedBoundingBox2D body_obb    # OBB around entire instrument body
tracking_pkg/OrientedBoundingBox2D handle_obb  # OBB around handle region

# 3D positions in world frame (filled after depth projection)
geometry_msgs/Point body_center_3d          # Body OBB center in world frame
geometry_msgs/Point handle_center_3d        # Handle OBB center in world frame
bool has_3d                                 # True if 3D projection succeeded
