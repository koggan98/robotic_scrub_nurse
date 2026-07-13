#!/usr/bin/env python3
"""
Tray-Loch vermessen — Klick-Helfer.
==================================

Die Trays sind ITEM-Profilrahmen mit einem offenen LOCH in der Mitte. Ueber dem
Loch darf der Greifer unter die Tray-Oberflaeche tauchen und ein flaches
Instrument sicher fassen; ueber einem Profilbalken wuerde er crashen.

MoveIt modelliert die Tray-Oberflaeche NICHT und faengt dich hier NICHT ab. Das
Polygon, das du hier vermisst, ist die EINZIGE Absicherung. Sorgfaeltig klicken.

Ablauf
------
  1) Linksklick auf die vier INNENECKEN des Profilrahmens (umlaufend).
     Jeder Klick wird per Strahl-Ebenen-Schnitt auf die Tray-Ebene projiziert —
     mit exakt derselben Mathematik, die auch die Werkzeugerkennung benutzt.
  2) 'w' schreibt den YAML-Block auf stdout -> nach config/tray_geometry.yaml.
  3) Skript NEU STARTEN: das gespeicherte Polygon wird nun ins Live-Bild
     ZURUECKprojiziert (gruen). Es muss auf den echten Profilen liegen.
     Das ist der Check, der ArUco-Drift und eine angestossene Kamera auffliegen
     laesst — der Fehler, der sonst still in einen Crash laeuft.
  4) Danach physisch gegenpruefen mit tcp_probe.py (Ecken + Lochmitte anfahren,
     Greifer schliessen) BEVOR irgendein tiefer Griff scharfgeschaltet wird.

Tasten:  Linksklick = Ecke setzen | u = letzte zurueck | c = alles loeschen
         w = YAML ausgeben        | q = Ende

VORSICHT beim Reclaim-Tray: dessen Kamera schaut SEITLICH. Du siehst die
Innen-WAND des Profils, nicht nur seine Oberkante. Klickst du zu tief an der
Wand, landet der Strahl HINTER der echten Innenkante und das Loch wird zu gross
geschaetzt — genau der Fehler, der den Greifer ins Profil faehrt. Immer die
OBERE Innenkante klicken.
"""

import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

TRAYS = {
    'instrument': {
        'name': 'instrument_tray',
        'ns': '/tray_camera',
        'frame': 'tray_camera_color_optical_frame',
    },
    'reclaim': {
        'name': 'reclaim_tray',
        'ns': '/reclaim_tray_camera',
        'frame': 'reclaim_tray_camera_color_optical_frame',
    },
}

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'src', 'tracking_pkg', 'config', 'tray_geometry.yaml',
)


def tf_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    m = np.eye(4)
    m[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def pixel_to_world_on_plane(u, v, K, T_world_cam, plane_z):
    """Strahl durch das Pixel, geschnitten mit der Ebene z = plane_z (world).

    Identisch zu _pixel_to_world_on_plane() in tool_detection_node.py — bewusst,
    damit das Polygon im selben Bezugsrahmen liegt wie die Werkzeug-Greifpunkte.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if abs(fx) < 1e-9 or abs(fy) < 1e-9:
        return None
    ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)

    origin = T_world_cam[:3, 3]
    ray = T_world_cam[:3, :3] @ ray_cam
    n = np.linalg.norm(ray)
    if n < 1e-9 or abs(ray[2]) < 1e-9:
        return None
    ray /= n
    t = (plane_z - origin[2]) / ray[2]
    if t <= 0.0:
        return None
    return origin + t * ray


def world_to_pixel(p_world, K, T_world_cam):
    """Die Gegenrichtung — gibt es im Repo noch nicht. Fuer die Rueckprojektion."""
    Rwc, twc = T_world_cam[:3, :3], T_world_cam[:3, 3]
    p_cam = Rwc.T @ (np.asarray(p_world, dtype=float) - twc)
    if p_cam[2] <= 1e-6:          # hinter der Kamera
        return None
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return (int(round(fx * p_cam[0] / p_cam[2] + cx)),
            int(round(fy * p_cam[1] / p_cam[2] + cy)))


class TrayCalib(Node):
    def __init__(self, tray_key, plane_z, stored_polys, world_offset_xy):
        super().__init__('tray_opening_calib')
        cfg = TRAYS[tray_key]
        self.tray_name = cfg['name']
        self.cam_frame = cfg['frame']
        self.plane_z = plane_z
        self.stored_polys = stored_polys
        # Same systematic bias correction tool_detection_node applies. Without it
        # the polygon would sit in the camera's biased frame while the tool grasp
        # points sit in the corrected one, and the two would disagree by exactly
        # the bias — which is how the gripper ends up over a profile.
        self.off = np.array(world_offset_xy, dtype=float)

        self.bridge = CvBridge()
        self.image = None
        self.K = None
        self.corners = []          # world XYZ
        self.corners_px = []       # was angeklickt wurde

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image, f"{cfg['ns']}/color/image_raw", self._on_image, 10)
        self.create_subscription(
            CameraInfo, f"{cfg['ns']}/color/camera_info", self._on_info, 10)

    def _on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def T_world_cam(self):
        try:
            return tf_to_matrix(self.tf_buffer.lookup_transform(
                'world', self.cam_frame, rclpy.time.Time()))
        except Exception:
            return None

    def on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        T = self.T_world_cam()
        if self.K is None or T is None:
            self.get_logger().warn('Noch keine camera_info / TF — Klick ignoriert.')
            return
        p = pixel_to_world_on_plane(x, y, self.K, T, self.plane_z)
        if p is None:
            self.get_logger().warn('Strahl trifft die Ebene nicht — Klick ignoriert.')
            return
        p[:2] += self.off          # in denselben korrigierten Frame wie die Werkzeuge
        self.corners.append(p)
        self.corners_px.append((x, y))
        print(f'  Ecke {len(self.corners)}: Pixel ({x:4d},{y:4d}) '
              f'-> world [{p[0]:+.4f}, {p[1]:+.4f}]  (z={p[2]:.4f})')

    def yaml_snippet(self):
        pts = ', '.join(f'[{p[0]:.4f}, {p[1]:.4f}]' for p in self.corners)
        return (f'  {self.tray_name}:\n'
                f'    plane_z: {self.plane_z}\n'
                f'    openings:\n'
                f'      - name: main\n'
                f'        polygon: [{pts}]\n'
                f'    edge_margin_m: 0.008')


def draw(node, frame):
    T = node.T_world_cam()

    # Gespeichertes Polygon zurueckprojizieren (gruen) — der Drift-Check.
    # Das Polygon steht im KORRIGIERTEN Frame, die Kamera-Projektion ist aber die
    # unkorrigierte. Also den Offset erst wieder herausrechnen, sonst laege die
    # gruene Kontur um genau den Bias daneben und man wuerde eine Drift sehen,
    # die gar keine ist.
    if T is not None and node.K is not None:
        for poly in node.stored_polys:
            pts = [world_to_pixel([x - node.off[0], y - node.off[1], node.plane_z],
                                  node.K, T)
                   for x, y in poly]
            pts = [p for p in pts if p is not None]
            if len(pts) >= 2:
                cv2.polylines(frame, [np.array(pts, np.int32)], True,
                              (0, 255, 0), 2)
                cv2.putText(frame, 'gespeichert', pts[0],
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Frisch geklickte Ecken (gelb)
    for i, px in enumerate(node.corners_px):
        cv2.circle(frame, px, 5, (0, 255, 255), -1)
        cv2.putText(frame, str(i + 1), (px[0] + 8, px[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    if len(node.corners_px) >= 2:
        cv2.polylines(frame, [np.array(node.corners_px, np.int32)],
                      len(node.corners_px) >= 3, (0, 255, 255), 2)

    status = 'TF+K ok' if (T is not None and node.K is not None) else 'WARTE auf TF/camera_info'
    color = (0, 255, 0) if 'ok' in status else (0, 0, 255)
    cv2.putText(frame, f'{node.tray_name}  z={node.plane_z:+.3f}  {status}',
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, 'Klick=Ecke  u=zurueck  c=clear  w=YAML  q=Ende',
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return frame


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tray', choices=sorted(TRAYS), required=True)
    ap.add_argument('--yaml', default=DEFAULT_YAML,
                    help='tray_geometry.yaml (fuer plane_z + Rueckprojektion)')
    args = ap.parse_args()

    tray_name = TRAYS[args.tray]['name']
    plane_z, stored, offset = 0.0, [], [0.0, 0.0]
    try:
        with open(args.yaml) as f:
            cfg = (yaml.safe_load(f) or {}).get('trays', {}).get(tray_name, {})
        plane_z = float(cfg.get('plane_z', 0.0))
        stored = [o.get('polygon', []) for o in (cfg.get('openings') or [])]
        offset = [float(v) for v in cfg.get('world_offset_xy', [0.0, 0.0])]
    except Exception as e:
        print(f'WARNUNG: {args.yaml} nicht lesbar ({e}) — plane_z=0.0')

    print(f'\nTray  : {tray_name}')
    print(f'Ebene : z = {plane_z:+.4f}  (muss fixed_tool_plane_z_m entsprechen)')
    print(f'Bias  : world_offset_xy = [{offset[0]:+.4f}, {offset[1]:+.4f}] '
          f'-> wird auf jeden Klick addiert (wie in tool_detection_node)')
    print(f'Bereits gespeichert: {len(stored)} Polygon(e) -> werden gruen zurueckprojiziert')
    if args.tray == 'reclaim':
        print('ACHTUNG: Seitenkamera. Immer die OBERE Innenkante des Profils klicken,')
        print('         sonst wird das Loch zu gross geschaetzt.')
    print()

    rclpy.init()
    node = TrayCalib(args.tray, plane_z, stored, offset)
    win = f'tray_opening_calib [{tray_name}]'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, node.on_click)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.image is None:
                continue
            cv2.imshow(win, draw(node, node.image.copy()))
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord('u') and node.corners:
                node.corners.pop()
                node.corners_px.pop()
                print(f'  zurueck -> {len(node.corners)} Ecken')
            if k == ord('c'):
                node.corners.clear()
                node.corners_px.clear()
                print('  geloescht')
            if k == ord('w'):
                if len(node.corners) < 3:
                    print('  Mindestens 3 Ecken noetig.')
                else:
                    print('\n--- nach config/tray_geometry.yaml, unter trays: ---')
                    print(node.yaml_snippet())
                    print('---------------------------------------------------\n')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
