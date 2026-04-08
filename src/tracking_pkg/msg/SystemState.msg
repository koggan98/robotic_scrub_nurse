# Overall system state for the world model
# Published by the world model node

std_msgs/Header header

# State machine
string state                                # IDLE, LISTENING, PICKING, TRANSPORTING, HANDOVER, RELEASING, RETURNING
string active_tool_id                       # Currently held tool (empty if none)
string active_tool_class                    # Class of held tool

# Available tools
tracking_pkg/GraspCandidate[] tool_candidates   # Current grasp candidates

# Hand tracking
tracking_pkg/HandState target_hand          # Current target hand state
bool hand_available                         # Whether a valid hand target exists

# Robot state
geometry_msgs/PoseStamped tcp_pose          # Current TCP pose
float32[6] joint_positions                  # Current joint positions
bool robot_ready                            # Whether robot is ready for commands
