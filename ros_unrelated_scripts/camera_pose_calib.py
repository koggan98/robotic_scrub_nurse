#!/usr/bin/env python3
"""
Kamera-Pose-Kalibrierung per Roboter-FK + PnP.

Warum: die Tray-Kamera wird per ArUco-Marker pro Launch neu lokalisiert, und dieser
Lock driftet ~cm (Reflexe, schräger Marker-Blick) — jede feste Greif-Kalibrierung
bricht damit beim nächsten Start. Die Kamera ist aber festgeschraubt. Also messen wir
ihre Pose EINMAL genau ein — mit dem Roboter als Referenz (nicht dem mehrdeutigen
Marker) — und verdrahten sie fest.

Prinzip: der Greifer-TCP (gripper_tip_link) ist per FK exakt im world-Frame bekannt.
Tippt man damit bekannte Punkte auf dem Tray an und klickt dieselben Punkte im
Kamerabild, liefert cv2.solvePnP(world_punkte, pixel, K, D) die Kamera-Pose. Die
Distortion (plumb_bob-d) geht dabei korrekt ein.

Ablauf:
  1. Greifer (geschlossen) auf einen gut erkennbaren Punkt auf dem Tray setzen
     (Ecken der Profil-Öffnung eignen sich am besten).
  2. Im Fenster 'c' drücken -> nimmt die aktuelle TCP-Weltposition auf.
  3. Greifer wegfahren, denselben Punkt im Bild anklicken.
  4. 3-8 mal wiederholen, gut über das Tray verteilt (die 4 Öffnungsecken + ein
     paar dazwischen).
  5. 's' -> solvePnP; gibt die feste Pose (world -> camera) als YAML-Zeile aus, plus
     den Reprojektionsfehler. Diese Zeile nach config/aruco_markers.yaml, Marker 110,
     unter detection: hardcoded_world_pose.
  6. Danach: den Jetson neu starten. Der ArUco-Re-Lock für diese Kamera entfällt,
     die feste Pose wird publiziert.

Nur lesend am ROS-Graph (außer dem Fenster). Kein Rebuild nötig — nur sourcen.
Tasten:  c=TCP aufnehmen  Klick=Pixel  u=letztes Paar zurück  s=solvePnP  q=Ende
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation as R
import tf2_ros

CAM_NS = '/tray_camera'
CAM_FRAME = 'tray_camera_color_optical_frame'
TCP_FRAME = 'gripper_tip_link'
WORLD = 'world'


class CamCalib(Node):
    def __init__(self):
        super().__init__('camera_pose_calib')
        self.bridge = CvBridge()
        self.image = None
        self.K = None
        self.D = None
        self.pending = None          # TCP-Weltpunkt, wartet auf den Pixel-Klick
        self.pairs = []              # Liste von (world_xyz(3,), pixel_uv(2,))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, f'{CAM_NS}/color/image_raw', self._on_image, 10)
        self.create_subscription(CameraInfo, f'{CAM_NS}/color/camera_info', self._on_info, 10)

    def _on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.D = np.array(msg.d, dtype=float).reshape(1, -1)

    def tcp_world(self):
        try:
            t = self.tf_buffer.lookup_transform(
                WORLD, TCP_FRAME, rclpy.time.Time()).transform.translation
            return np.array([t.x, t.y, t.z], dtype=float)
        except Exception:
            return None

    def capture(self):
        w = self.tcp_world()
        if w is None:
            print('  Keine TF world -> gripper_tip_link. Läuft der Roboter/State-Publisher?')
            return
        self.pending = w
        print(f"  TCP aufgenommen: [{w[0]:+.4f}, {w[1]:+.4f}, {w[2]:+.4f}] "
              f"— Greifer wegfahren und den Punkt im Bild klicken.")

    def on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.pending is None:
            print("  Erst mit 'c' die TCP-Position aufnehmen, dann klicken.")
            return
        self.pairs.append((self.pending.copy(), np.array([x, y], dtype=float)))
        w = self.pending
        print(f"  Paar {len(self.pairs)}: world [{w[0]:+.4f}, {w[1]:+.4f}, {w[2]:+.4f}] "
              f"<-> pixel ({x}, {y})")
        self.pending = None

    def undo(self):
        if self.pairs:
            self.pairs.pop()
            print(f'  letztes Paar entfernt ({len(self.pairs)} übrig).')

    def _solvepnp(self, obj, img):
        """PnP über die (nahezu) koplanaren Tray-Punkte. IPPE ist der planare Löser
        (stabil bei koplanaren Punkten); Fallback SQPNP / ITERATIVE."""
        for flag in ('SOLVEPNP_IPPE', 'SOLVEPNP_SQPNP', 'SOLVEPNP_ITERATIVE'):
            f = getattr(cv2, flag, None)
            if f is None:
                continue
            try:
                ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.D, flags=f)
                if ok:
                    return rvec, tvec, flag
            except cv2.error:
                continue
        return None, None, None

    def solve(self):
        if len(self.pairs) < 4:
            print(f'  Mindestens 4 Punkte nötig (aktuell {len(self.pairs)}).')
            return
        if self.K is None:
            print('  Noch keine camera_info empfangen.')
            return
        obj = np.array([p[0] for p in self.pairs], dtype=np.float64)
        img = np.array([p[1] for p in self.pairs], dtype=np.float64)
        rvec, tvec, flag = self._solvepnp(obj, img)
        if rvec is None:
            print('  solvePnP fehlgeschlagen.')
            return
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)

        # solvePnP liefert world_in_camera (Abbildung world -> cam). Wir wollen die
        # Kamera-Pose IM world-Frame (world -> camera): das Inverse.
        Rwc, _ = cv2.Rodrigues(rvec)
        Rcw = Rwc.T
        tcw = (-Rcw @ tvec.reshape(3))
        quat = R.from_matrix(Rcw).as_quat()   # xyzw

        print('\n' + '=' * 68)
        print(f'  PnP ({flag}, {len(self.pairs)} Punkte)  '
              f'Reprojektion: mean {err.mean():.1f} px, max {err.max():.1f} px')
        print('  Kamera im Weltframe:  '
              f't=[{tcw[0]:+.4f}, {tcw[1]:+.4f}, {tcw[2]:+.4f}]  '
              f'(ArUco-Lock war ~[0.00, 0.49, 0.42] zum Vergleich)')
        print('\n  -> in config/aruco_markers.yaml, Marker 110, unter detection: '
              'einfügen:')
        print(f'      hardcoded_world_pose: [{tcw[0]:.5f}, {tcw[1]:.5f}, {tcw[2]:.5f}, '
              f'{quat[0]:.5f}, {quat[1]:.5f}, {quat[2]:.5f}, {quat[3]:.5f}]')
        print('=' * 68 + '\n')
        if err.max() > 5.0:
            print('  WARNUNG: Reprojektionsfehler >5 px an einem Punkt — evtl. ein Klick '
                  'daneben. Mit "u" das Paar entfernen und neu, oder mehr Punkte nehmen.\n')


def draw(node, frame):
    cv2.putText(frame, f'Punkte: {len(node.pairs)}   '
                f'{"TCP aufgenommen - jetzt Pixel klicken" if node.pending is not None else "c=TCP aufnehmen"}',
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255) if node.pending is not None else (0, 255, 0), 2)
    cv2.putText(frame, 'c=TCP  Klick=Pixel  u=zurueck  s=solvePnP  q=Ende',
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    ok = 'K ok' if node.K is not None else 'WARTE auf camera_info'
    cv2.putText(frame, ok, (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 0) if node.K is not None else (0, 0, 255), 1)
    return frame


def main():
    rclpy.init()
    node = CamCalib()
    win = 'camera_pose_calib [tray_camera]'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, node.on_click)
    print('\nGreifer auf einen Tray-Punkt setzen -> "c" -> wegfahren -> Pixel klicken. '
          '4-8 Punkte, dann "s".\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.image is None:
                continue
            cv2.imshow(win, draw(node, node.image.copy()))
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('c'):
                node.capture()
            elif k == ord('u'):
                node.undo()
            elif k == ord('s'):
                node.solve()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
