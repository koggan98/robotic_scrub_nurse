# Computed grasp candidate for a specific tool
# Published by the grasp reasoning node

std_msgs/Header header

string tool_id                              # References ToolDetection.tool_id
string tool_class                           # Tool class for context
string location                             # Which tray: "instrument_tray" | "reclaim_tray"

# Grasp geometry in world frame
geometry_msgs/PoseStamped grasp_pose        # Where to grasp (handle centroid + approach orientation)
geometry_msgs/Vector3 approach_direction     # Direction to approach from (typically top-down)
geometry_msgs/Vector3 functional_end_dir    # Points from handle toward functional end
geometry_msgs/Vector3 handle_axis           # Long axis of the handle

# The TOOL's pose, as opposed to the GRASP pose. These differ, and the difference
# matters: grasp_pose = handle_center + d * functional_end_dir, and d depends on
# the tray (the sliding-into-the-opening strategy uses that tray's hole). So a tool
# grasped on the reclaim tray is held at a DIFFERENT point along its shaft than the
# same tool was on the instrument tray. To put it back where it belongs you must
# reconstruct the tool pose, not replay the grasp pose — otherwise it lands
# displaced along its own axis by (d_reclaim - d_home).
geometry_msgs/Point handle_center           # Handle centroid in world frame

# Handover semantics (derived from tool knowledge base)
string handover_rule                        # e.g. "functional_end_away", "blade_away", "neutral"
geometry_msgs/PoseStamped handover_pose     # Suggested handover pose relative to hand
float32 grasp_confidence                    # Combined detection + geometry confidence

# Grasp strategy (filled by grasp_geometry_node from tool_knowledge_base.yaml's
# grasp_strategies block + tray_geometry.yaml). Lets you see from `ros2 topic echo`
# WHY a tool is being grasped where and how deep it is.
float32 grasp_z_offset                      # Class/strategy z delta vs. the tray plane.
                                            # Negative = below the tray surface, which is
                                            # only ever emitted when over_opening is true.
bool over_opening                           # Grasp point AND both fingertips confirmed
                                            # inside a measured tray opening
string grasp_strategy                       # "fixed" | "sliding_opening" | "sliding_no_opening"

# Semantic enrichment (filled by tool_semantics_node from tool_knowledge_base.yaml).
# Upstream producers (grasp_geometry_node, world_model_builder) leave these empty/zero.
string display_name                         # e.g. "Needle Holder"
string handover_description                 # e.g. "Hand over with jaws away from surgeon"
string grip_strategy                        # "parallel" | "pinch" | "wrap"
string functional_end_label                 # "jaws" | "blade" | "tips" | "hook" | "blades"
float32 lift_height                         # meters above tray surface
