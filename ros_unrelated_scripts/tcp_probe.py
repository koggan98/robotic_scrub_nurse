#!/usr/bin/env python3
"""
TCP-Sonde — faehrt den Greifer-TCP (gripper_tip_link) auf eine Weltkoordinate.

Zweck: die Projektionsebene der Werkzeugerkennung verifizieren. Die Perzeption
legt JEDEN erkannten Greifpunkt per Strahl-Ebenen-Schnitt exakt auf eine feste
z-Ebene (fixed_tool_plane_z_m):

    Instrument-Tray:  z = +0.044
    Reclaim-Tray:     z = -0.137

Faehrt man den TCP genau dorthin und SCHLIESST den Greifer, muessen die
Fingerspitzen die Tray-Oberflaeche beruehren. Tun sie das nicht, ist die Ebene
falsch:
    ros2 topic pub --once /gripper_mover std_msgs/msg/Bool "{data: false}"   # zu
    ros2 topic pub --once /gripper_mover std_msgs/msg/Bool "{data: true}"    # auf

Die Ebene ist KEIN reiner Hoehenregler: weil die Reclaim-Kamera seitlich schaut,
verschiebt sie auch das x/y des Greifpunkts. Schau beim Antasten also auch, ob
die Fingerspitzen mittig auf dem Werkzeuggriff landen.

Redet direkt mit MoveIt (/move_action). Kein Rebuild noetig — nur Workspace
sourcen. Plant per Default nur (Trockenlauf); Bewegung erst mit --go.

Beispiele
---------
  # Trockenlauf: Ebene des Reclaim-Trays, Mitte der Auflage
  python3 tcp_probe.py --plane reclaim

  # Wirklich hinfahren, vorher 5 cm darueber schweben
  python3 tcp_probe.py --plane reclaim --dz 0.05 --go
  python3 tcp_probe.py --plane reclaim --go

  # Auf den Greifpunkt eines KONKRET erkannten Werkzeugs (aus dem Weltmodell).
  # Das ist der ehrlichste Test: exakt die Pose, die der Roboter greifen wuerde.
  python3 tcp_probe.py --tool reclaim_0 --go

  # Beliebiger Punkt
  python3 tcp_probe.py --x 0.245 --y -0.36 --z -0.145 --go

  # Aktuelle TCP-Hoehe nur ablesen (faehrt nichts):
  ros2 run tf2_ros tf2_echo world gripper_tip_link
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation as R
from tracking_msgs.srv import GetWorldState

# Muss zu nuc_launch.py / skill_executor_node.cpp passen.
GROUP = 'ur_manipulator'
EEF_LINK = 'gripper_tip_link'
WORLD = 'world'
TOOL_YAW_OFFSET_RAD = 1.57079632679   # skill_executor: tool_yaw_offset_rad

# fixed_tool_plane_z_m aus jetson_launch.py — mit diesem Skript + geschlossenem
# Greifer gegen die Tray-Oberflaeche gemessen.
PLANES = {
    'instrument': 0.044,
    'reclaim': -0.137,
}
# Ein Punkt AUF der jeweiligen Ablageflaeche (nicht zwingend die Mitte).
# Reclaim: reclaim_tray_collision_publisher ORIGIN_XYZ + Segmente.
# Instrument: an einem real erkannten Werkzeug abgelesen.
PLANE_CENTER_XY = {
    'instrument': (0.30, 0.33),
    'reclaim': (0.245, -0.36),
}


def top_down_quat(handle_axis, yaw_offset=TOOL_YAW_OFFSET_RAD):
    """Identisch zu topDownQuaternionFromHandleAxis() im skill_executor:
    Greifer-Z zeigt gerade nach unten, Greifer-X ist die um yaw_offset gedrehte
    Werkzeugachse (in die XY-Ebene geklappt)."""
    ax = np.array([handle_axis[0], handle_axis[1], 0.0], dtype=float)
    n = float(np.linalg.norm(ax))
    ax = np.array([1.0, 0.0, 0.0]) if n < 1e-6 else ax / n

    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    x_g = np.array([c * ax[0] - s * ax[1], s * ax[0] + c * ax[1], 0.0])
    z_g = np.array([0.0, 0.0, -1.0])
    y_g = np.cross(z_g, x_g)
    quat = R.from_matrix(np.column_stack([x_g, y_g, z_g])).as_quat()  # x,y,z,w
    return [float(v) for v in quat]


class TcpProbe(Node):
    def __init__(self):
        super().__init__('tcp_probe')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.world_state = self.create_client(GetWorldState, '/get_world_state')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ── Weltmodell ───────────────────────────────────────────────────
    def lookup_tool(self, tool_id):
        if not self.world_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/get_world_state nicht da — laeuft der Jetson?')
            return None
        fut = self.world_state.call_async(GetWorldState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error('/get_world_state Timeout')
            return None
        cands = fut.result().world_state.tool_candidates
        if not cands:
            self.get_logger().error('Weltmodell kennt keine Werkzeuge')
            return None
        for c in cands:
            if c.tool_id == tool_id:
                return c
        ids = ', '.join(f'{c.tool_id} ({c.tool_class}, {c.location})' for c in cands)
        self.get_logger().error(f"tool_id '{tool_id}' unbekannt. Verfuegbar: {ids}")
        return None

    # ── TF ───────────────────────────────────────────────────────────
    def tcp_now(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD, EEF_LINK, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    # ── Bewegung ─────────────────────────────────────────────────────
    def goto(self, pose, vel, accel, plan_only):
        if not self.move.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action nicht da — laeuft move_group?')
            return False

        pos = PositionConstraint()
        pos.header.frame_id = WORLD
        pos.link_name = EEF_LINK
        pos.target_point_offset = Vector3()
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.001]          # 1 mm — wir wollen exakt dorthin
        pos.constraint_region.primitives.append(region)
        pos.constraint_region.primitive_poses.append(pose)
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = WORLD
        ori.link_name = EEF_LINK
        ori.orientation = pose.orientation
        ori.absolute_x_axis_tolerance = 0.01
        ori.absolute_y_axis_tolerance = 0.01
        ori.absolute_z_axis_tolerance = 0.01
        ori.weight = 1.0

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.start_state.is_diff = True
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = accel
        c = Constraints()
        c.position_constraints.append(pos)
        c.orientation_constraints.append(ori)
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False

        fut = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('MoveGroup hat das Ziel abgelehnt')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        code = res_fut.result().result.error_code.val
        if code != 1:   # moveit_msgs/MoveItErrorCodes.SUCCESS
            self.get_logger().error(
                f'Planen/Fahren fehlgeschlagen, error_code={code} '
                '(-1=FAILURE, -6=NO_IK_SOLUTION, -12=START_STATE_IN_COLLISION, '
                '-31=GOAL_IN_COLLISION)')
            return False
        return True


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plane', choices=sorted(PLANES),
                   help='Auf die Projektionsebene dieses Trays, Mitte der Ablage')
    p.add_argument('--tool', help='tool_id aus dem Weltmodell, z.B. reclaim_0')
    p.add_argument('--x', type=float)
    p.add_argument('--y', type=float)
    p.add_argument('--z', type=float)
    p.add_argument('--dz', type=float, default=0.0,
                   help='Zusaetzlicher Hoehenversatz in m (z.B. 0.05 = 5 cm darueber schweben)')
    p.add_argument('--vel', type=float, default=0.1, help='Geschwindigkeit 0..1 (Default 0.1)')
    p.add_argument('--go', action='store_true',
                   help='WIRKLICH FAHREN. Ohne dieses Flag wird nur geplant.')
    args = p.parse_args()

    rclpy.init()
    node = TcpProbe()

    handle_axis = (1.0, 0.0, 0.0)
    source = ''

    if args.tool:
        cand = node.lookup_tool(args.tool)
        if cand is None:
            rclpy.shutdown()
            return 1
        gp = cand.grasp_pose.pose.position
        x, y, z = gp.x, gp.y, gp.z
        handle_axis = (cand.handle_axis.x, cand.handle_axis.y, cand.handle_axis.z)
        source = f'Werkzeug {cand.tool_id} ({cand.tool_class}) auf {cand.location}'
    elif args.plane:
        x, y = PLANE_CENTER_XY[args.plane]
        z = PLANES[args.plane]
        source = f'Projektionsebene {args.plane} (fixed_tool_plane_z_m)'
    elif None not in (args.x, args.y, args.z):
        x, y, z = args.x, args.y, args.z
        source = 'freie Koordinate'
    else:
        p.error('Entweder --plane, --tool oder --x/--y/--z angeben.')

    z += args.dz

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    q = top_down_quat(handle_axis)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = q

    before = node.tcp_now()
    print(f'\nQuelle : {source}')
    if before:
        print(f'TCP ist: x={before[0]:+.4f}  y={before[1]:+.4f}  z={before[2]:+.4f}')
    print(f'TCP Ziel: x={pose.position.x:+.4f}  y={pose.position.y:+.4f}  '
          f'z={pose.position.z:+.4f}   (dz={args.dz:+.3f})')
    print(f'Modus  : {"FAHREN" if args.go else "nur planen (Trockenlauf)"}   vel={args.vel}\n')

    ok = node.goto(pose, args.vel, args.vel, plan_only=not args.go)

    if ok and args.go:
        rclpy.spin_once(node, timeout_sec=1.0)
        after = node.tcp_now()
        if after:
            print(f'\nTCP jetzt: x={after[0]:+.4f}  y={after[1]:+.4f}  z={after[2]:+.4f}')
            print(f'Abweichung zum Ziel: {(after[2]-pose.position.z)*1000:+.1f} mm in z')
    print('\nOK' if ok else '\nFEHLGESCHLAGEN')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
