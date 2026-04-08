# Hand state from scene perception (side camera)
# Replaces the old gesture-triggered hand_pose

std_msgs/Header header

geometry_msgs/PoseStamped hand_pose         # Palm center in world frame
float32 confidence                          # Tracking confidence [0.0, 1.0]
bool is_tracked                             # Whether the hand is currently tracked
string hand_id                              # Tracking ID for multi-hand scenarios

# Keypoint positions in world frame (subset for release logic)
geometry_msgs/Point wrist
geometry_msgs/Point index_tip
geometry_msgs/Point middle_tip
geometry_msgs/Point palm_center
