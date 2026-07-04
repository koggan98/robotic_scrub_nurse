# Execution plan issued by the LLM node
# Consumed by the motion execution node

std_msgs/Header header

string action                               # "pick_and_handover", "retrieve", "return_home", "abort"
string tool_id                              # Target tool ID from world model
string tool_class                           # Tool class for logging

# Pick parameters
geometry_msgs/PoseStamped grasp_pose        # Where to pick the tool
geometry_msgs/Vector3 approach_direction     # Approach direction for pick

# Handover parameters
string handover_rule                        # "functional_end_away", "blade_away", "neutral"
geometry_msgs/PoseStamped handover_pose     # Target handover pose (updated dynamically)
float32 release_force_threshold             # Force threshold for release [N]

# LLM reasoning trace (for logging/debugging)
string reasoning                            # Brief explanation of LLM decision
