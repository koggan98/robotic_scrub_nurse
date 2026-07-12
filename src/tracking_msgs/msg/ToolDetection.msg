# Single detected tool on one of the trays
# Published by the tray perception nodes (one instance per tray)

std_msgs/Header header

string tool_id                              # Persistent tracking ID (e.g. "tool_3")
string tool_class                           # Class name from OBB model (e.g. "needle_holder")
string location                             # Which tray: "instrument_tray" | "reclaim_tray"
float32 confidence                          # Detection confidence [0.0, 1.0]

tracking_msgs/OrientedBoundingBox2D body_obb    # OBB around entire instrument body
tracking_msgs/OrientedBoundingBox2D handle_obb  # OBB around handle region

# 3D positions in world frame (filled after depth projection)
geometry_msgs/Point body_center_3d          # Body OBB center in world frame
geometry_msgs/Point handle_center_3d        # Handle OBB center in world frame
bool has_3d                                 # True if 3D projection succeeded
