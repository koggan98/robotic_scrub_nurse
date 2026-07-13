# A transition in a tool's life that PERCEPTION CANNOT SEE.
#
# The world model can watch a tool sitting on a tray, but it cannot see the moment
# the surgeon pulls it out of the gripper's jaws — and it cannot tell that moment
# apart from the robot simply opening the gripper. /system_state_update is no help:
# its RELEASING state is published by three different code paths.
#
# An instrument count whose whole purpose is reliability must not rest on that kind
# of inference. So the executor says explicitly what happened.

std_msgs/Header header

string event                        # PICKED | HANDED_OVER | RELEASED | PLACED_HOME | DROPPED
string track_id                     # world-model track id (tool_3 / reclaim_1) at the time
string tool_class
string from_location                # "instrument_tray" | "reclaim_tray"

# The tool's pose at the moment of the event, and where along it the gripper holds
# it. Needed to put it back correctly — see GraspCandidate.handle_center.
geometry_msgs/Point handle_center
geometry_msgs/Vector3 functional_end_dir
float32 grasp_distance_m
