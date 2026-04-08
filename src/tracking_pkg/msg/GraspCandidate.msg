# Computed grasp candidate for a specific tool
# Published by the grasp reasoning node

std_msgs/Header header

string tool_id                              # References ToolDetection.tool_id
string tool_class                           # Tool class for context

# Grasp geometry in world frame
geometry_msgs/PoseStamped grasp_pose        # Where to grasp (handle centroid + approach orientation)
geometry_msgs/Vector3 approach_direction     # Direction to approach from (typically top-down)
geometry_msgs/Vector3 functional_end_dir    # Points from handle toward functional end
geometry_msgs/Vector3 handle_axis           # Long axis of the handle

# Handover semantics (derived from tool knowledge base)
string handover_rule                        # e.g. "functional_end_away", "blade_away", "neutral"
geometry_msgs/PoseStamped handover_pose     # Suggested handover pose relative to hand
float32 grasp_confidence                    # Combined detection + geometry confidence
