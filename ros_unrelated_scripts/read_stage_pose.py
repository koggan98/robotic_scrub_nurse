#!/usr/bin/env python3
"""
Stage-Pose ablesen — liest die aktuelle Armstellung und druckt sie fertig zum
Einfuegen in nuc_launch.py / llm_launch.py.

Wozu: der Tray-Korridor im skill_executor aendert die HOEHE nur an einer einzigen
getaughten Stelle pro Tray (der "Stage-Pose") und faehrt seitwaerts nur in einer
Fahrspur ueber den Werkzeugen. Wo diese Stelle liegt, entscheidest DU, indem du
den Arm dorthin faehrst — die Fahrspur folgt daraus per Vorwaertskinematik.
Es ist keine einzige Koordinate im Code hartkodiert.

  instrument_stage_joints  ueber dem -x-Ende des Instrument-Trays
  reclaim_stage_joints     ueber dem Reclaim-Tray, ein Stueck INNERHALB der
                           Oeffnung vom -x-Ende her — NICHT in der Ecke, dort
                           steht der 60-cm-Kamerapfosten

ZWEI FALLEN, die dieses Skript fuer dich abraeumt:

1. Der UR-Treiber publiziert /joint_states NICHT in der Reihenfolge, die der
   skill_executor erwartet. Ein blindes `ros2 topic echo /joint_states` liefert
   sechs Zahlen in falscher Ordnung — der Arm faehrt dann woanders hin.
   Hier werden sie nach joint_state_names sortiert (shoulder_pan, shoulder_lift,
   elbow, wrist_1, wrist_2, wrist_3).

2. Die Stage-Pose muss WIRKLICH ueber dem Tray stehen und der Weg nach oben muss
   frei sein. Deshalb wird auch der TCP in Weltkoordinaten gezeigt, samt Abstand
   zu Pfosten und Halterung — damit du siehst, ob die Stelle taugt, bevor du sie
   uebernimmst.

Nur lesend. Bewegt nichts. Kein Rebuild noetig — nur Workspace sourcen.

Benutzung:
    # Arm von Hand / per Teach-Pendant / per RViz dorthin fahren, dann:
    python3 ros_unrelated_scripts/read_stage_pose.py
    python3 ros_unrelated_scripts/read_stage_pose.py --name reclaim_stage_joints
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

CANONICAL = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Aus config/tray_geometry.yaml bzw. reclaim_tray_collision_publisher.py.
INSTRUMENT_OPENING_X = (-0.348, 0.317)
INSTRUMENT_PLANE_Z = 0.044
RECLAIM_OPENING_X = (0.153, 0.370)
RECLAIM_PLANE_Z = -0.137
POST40_X = (0.125, 0.165)      # der 60-cm-Kamerapfosten am Reclaim-Tray
TOP_BAR_X_MAX = 0.110          # Halterung: Querbalken + Abwurfkante enden hier
TOP_BAR_Z_MAX = 0.045          # Oberkante des Querbalkens (nach der Verdickung)


class StageReader(Node):
    def __init__(self):
        super().__init__("read_stage_pose")
        self.joints = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_joints(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        if all(n in by_name for n in CANONICAL):
            self.joints = [by_name[n] for n in CANONICAL]

    def tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "world", "gripper_tip_link", rclpy.time.Time()).transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None


def assess(tcp):
    """Sagt, ob die Stelle als Stage-Pose taugt — und warum nicht."""
    x, y, z = tcp
    notes = []
    if y > 0:
        tray, (x_lo, x_hi), plane = "Instrument", INSTRUMENT_OPENING_X, INSTRUMENT_PLANE_Z
    else:
        tray, (x_lo, x_hi), plane = "Reclaim", RECLAIM_OPENING_X, RECLAIM_PLANE_Z

    notes.append(f"Ueber dem {tray}-Tray (Ebene z = {plane:+.3f}).")
    if not (x_lo <= x <= x_hi):
        notes.append(
            f"WARNUNG: x = {x:+.3f} liegt AUSSERHALB der Oeffnung "
            f"[{x_lo:+.3f}, {x_hi:+.3f}]. Die Fahrspur startet dann neben dem Tray.")
    else:
        notes.append(
            f"x = {x:+.3f} liegt in der Oeffnung [{x_lo:+.3f}, {x_hi:+.3f}], "
            f"{(x - x_lo) * 1000:.0f} mm vom -x-Rand.")

    height = z - plane
    notes.append(f"TCP steht {height * 1000:.0f} mm ueber der Tray-Ebene.")
    if height < 0.12:
        notes.append(
            "WARNUNG: das ist niedrig fuer eine Parkpose. Sie sollte klar ueber "
            "der Fahrspur liegen (Fahrspur = Ebene + 50 mm).")

    if tray == "Reclaim":
        if POST40_X[0] <= x <= POST40_X[1]:
            notes.append(
                f"GEFAHR: x = {x:+.3f} liegt genau im Kamerapfosten "
                f"[{POST40_X[0]:+.3f}, {POST40_X[1]:+.3f}]. Hier kann der Arm nicht "
                f"senkrecht hoch. Weiter nach +x.")
        else:
            gap = (x - POST40_X[1]) * 1000
            notes.append(f"Abstand zum Kamerapfosten: {gap:.0f} mm in +x.")
        if z < TOP_BAR_Z_MAX and x < TOP_BAR_X_MAX + 0.06:
            notes.append(
                "WARNUNG: nahe an der Halterung (Querbalken bis z = +0.045, "
                "x bis +0.110).")
    return notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="instrument_stage_joints",
                    help="Parametername fuer die Ausgabe "
                         "(instrument_stage_joints | reclaim_stage_joints)")
    args = ap.parse_args()

    rclpy.init()
    node = StageReader()
    for _ in range(200):                       # ~4 s auf joint_states + TF warten
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.joints is not None and node.tcp() is not None:
            break

    if node.joints is None:
        print("Keine /joint_states empfangen. Laeuft der UR-Treiber?", file=sys.stderr)
        return 1

    print("\n  " + args.name + ":")
    print("      [" + ", ".join(f"{v:.10f}" for v in node.joints[:3]) + ",")
    print("       " + ", ".join(f"{v:.10f}" for v in node.joints[3:]) + "],")
    print("\n  (Grad: " + ", ".join(f"{math.degrees(v):.1f}" for v in node.joints) + ")")

    tcp = node.tcp()
    if tcp is None:
        print("\n  Keine TF world -> gripper_tip_link. TCP-Pruefung uebersprungen.")
    else:
        print(f"\n  TCP im Weltframe: x={tcp[0]:+.3f}  y={tcp[1]:+.3f}  z={tcp[2]:+.3f}")
        for line in assess(tcp):
            print("    - " + line)
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
