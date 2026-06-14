#!/usr/bin/env python3

# gripper_mover true = gripper open
# gripper_mover false = gripper close
# gripper_zeroer true = sensing aktiv
# gripper_zeroer false = sensing inaktiv

# offsetten: node.reset_force_offset()
# in code offsetten: self.reset_force_offset()


import time
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Empty, Int32


class URCommand:
    def __init__(self, robot_ip, robot_command_port, gripper_port):
        self.robot_ip = robot_ip
        self.robot_command_port = robot_command_port
        self.gripper_port = gripper_port
        self.socket_ur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_ur.connect((self.robot_ip, self.robot_command_port))
        self.socket_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_gripper.connect((self.robot_ip, self.gripper_port))
        # Kurzer Timeout, damit recv beim Auslesen von GET-Antworten nicht blockiert.
        self.socket_gripper.settimeout(1.0)
        print('UR Node started.')

    def ur_command(self, command):
        full_command = f"def my_prog():\n{command}\nend\n"
        try:
            self.socket_ur.sendall(full_command.encode('utf-8'))
            print(f'Sent URScript command: {full_command}')
        except socket.error as e:
            print(f"Socket error: {e}")

    def gripper_command(self, command):
        self.socket_gripper.sendall(command.encode('utf-8'))
        #print(f'Sent Gripper command: {command}')
        # Jeder SET-/GTO-Befehl beantwortet der Robotiq-URCap-Socket mit "ack".
        # Diese Antwort konsumieren, damit der TCP-Puffer nicht mit alten Acks
        # vollläuft und spätere GET-Antworten nicht verfälscht werden.
        try:
            self.socket_gripper.recv(1024)
        except socket.timeout:
            pass

    def query_gripper_var(self, name):
        """Liest eine Gripper-Variable per "GET <name>" (z. B. OBJ, POS, PRE).

        Gibt den Integer-Wert zurück oder None bei Timeout/Parsefehler.
        Liest zeilenweise, bis eine Antwort mit <name> beginnt, um robust
        gegen evtl. noch gepufferte Acks zu sein.
        """
        try:
            self.socket_gripper.sendall(f'GET {name}\n'.encode('utf-8'))
        except socket.error as e:
            print(f"Gripper socket error on GET {name}: {e}")
            return None

        buffer = ''
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                chunk = self.socket_gripper.recv(1024).decode('utf-8', errors='ignore')
            except socket.timeout:
                break
            if not chunk:
                break
            buffer += chunk
            for line in buffer.splitlines():
                parts = line.strip().split()
                if len(parts) == 2 and parts[0] == name:
                    try:
                        return int(parts[1])
                    except ValueError:
                        return None
        return None

    def wait_for_gripper_stopped(self, timeout=3.0):
        """Wartet, bis die Greifbewegung abgeschlossen ist, und gibt gOBJ zurück.

        Robotiq meldet gOBJ=0, solange die Finger in Bewegung sind. Sobald sie
        stoppen (durch Objektkontakt oder an der Zielposition), wird gOBJ zu
        1/2/3. Eine kurze Startverzögerung stellt sicher, dass die Bewegung
        überhaupt begonnen hat (sonst läse man evtl. das alte gOBJ=3 vom
        vorherigen Befehl).
        """
        time.sleep(0.3)  # Bewegung erst anlaufen lassen
        deadline = time.time() + timeout
        obj = None
        while time.time() < deadline:
            obj = self.query_gripper_var("OBJ")
            if obj is not None and obj != 0:
                return obj
            time.sleep(0.05)
        return obj

    def command_gripper(self, position, speed=255, force=255):
        if 0 <= position <= 255 and 0 <= speed <= 255 and 0 <= force <= 255:
            command_pos = f'SET POS {position}\n'
            self.gripper_command(command_pos)
            command_speed = f'SET SPE {speed}\n'
            self.gripper_command(command_speed)
            command_force = f'SET FOR {force}\n'
            self.gripper_command(command_force)
            command_gto = 'SET GTO 1\n'
            self.gripper_command(command_gto)

            # Berechnung der Dauer für den Gripper-Vorgang
            time_for_speed = 4 - (3.25 * (speed - 1) / 254)
            time_for_position = (position / 255) * time_for_speed
            time_to_sleep = time_for_speed - time_for_position

            time.sleep(time_to_sleep)
        else:
            print('Invalid gripper command. Position, speed and force must be between 0 and 255.')

    def close_connections(self):
        self.socket_ur.close()
        self.socket_gripper.close()
        print('Closed connection to robot.')


class SocketControllerNode(Node):
    def __init__(self):
        super().__init__('socket_controller')
        self.robot_ip = "192.168.12.10"
        #self.robot_ip = "192.168.12.21"
        #self.robot_ip = "mirur_ur3e_eth"
        self.robot_command_port = 30002
        self.gripper_port = 63352
        self.ur_node = URCommand(self.robot_ip, self.robot_command_port, self.gripper_port)
        self.get_logger().info("Socket Mover Node initialized")
        self.declare_parameter("reclaim.close_speed", 255)
        self.declare_parameter("reclaim.close_force", 1)
        self.reclaim_close_speed = int(self.get_parameter("reclaim.close_speed").value)
        self.reclaim_close_force = int(self.get_parameter("reclaim.close_force").value)

        # Parameter für die Werkzeug-Greif-Erkennung (Robotiq gOBJ/gPO)
        # gOBJ trennt zuverlässig (2 = gegriffen, 3 = leer); der gPO-Cross-Check
        # ist standardmäßig aus, weil leer/Werkzeug bei gPO sehr nah beieinander
        # liegen und dünne Werkzeuge sonst fälschlich verworfen werden.
        self.declare_parameter("grasp_check.empty_close_pos", 230)
        self.declare_parameter("grasp_check.pos_margin", 10)
        self.declare_parameter("grasp_check.use_position_crosscheck", False)
        self.grasp_empty_close_pos = int(self.get_parameter("grasp_check.empty_close_pos").value)
        self.grasp_pos_margin = int(self.get_parameter("grasp_check.pos_margin").value)
        self.grasp_use_pos_crosscheck = bool(self.get_parameter("grasp_check.use_position_crosscheck").value)

        # Kontinuierliche Überwachung: pollt gOBJ, solange ein Werkzeug gehalten
        # wird, und meldet, wenn es verloren geht (gOBJ 2 -> 3).
        self.declare_parameter("grasp_check.monitor_enabled", True)
        self.declare_parameter("grasp_check.monitor_period", 0.5)
        self.grasp_monitor_enabled = bool(self.get_parameter("grasp_check.monitor_enabled").value)
        self.grasp_monitor_period = float(self.get_parameter("grasp_check.monitor_period").value)
        self.monitoring_active = False

        # publisher für gripper-status
        self.status_publisher = self.create_publisher(Bool, '/gripper_done', 10)
        self.gripper_position_done_publisher = self.create_publisher(Bool, '/gripper_position_done', 10)
        # Publisher für das Ergebnis der Werkzeug-Greif-Erkennung
        self.tool_grasped_publisher = self.create_publisher(Bool, '/tool_grasped', 10)

        # Offsets für Kraft und Drehmoment
        self.force_offset = None
        self.torque_offset = None

        # Bool für Node aktivierung
        self.zeroer_active_ = False

        # Subscriber für die Kraftsensor-Daten
        self.subscription = self.create_subscription(WrenchStamped,'/force_torque_sensor_broadcaster/wrench', self.force_callback, 10)

        # Subscriber für den Gripper-Befehl
        self.subscription2 = self.create_subscription(Bool, '/gripper_mover', self.gripper_mover_callback, 10)

        # Subscriber für den Gripper-Zeroer
        self.subscription3 = self.create_subscription(Bool, '/gripper_zeroer', self.gripper_zeroer_callback, 10)
        # Subscriber für explizite Positionskommandos
        self.subscription4 = self.create_subscription(Int32, '/gripper_position_command', self.gripper_position_callback, 10)
        # Subscriber für On-Demand-Frischprüfung des Greifzustands (z. B. nach Lift)
        self.subscription5 = self.create_subscription(Empty, '/verify_grasp', self.verify_grasp_callback, 10)

        # Timer für die kontinuierliche Werkzeug-Überwachung
        if self.grasp_monitor_enabled:
            self.monitor_timer = self.create_timer(self.grasp_monitor_period, self._monitor_grasp)

    def gripper_mover_callback(self, bool_msg):
        # Extrahiere bool aus nachricht
        gripper_state = bool_msg.data
        if gripper_state:
            self.get_logger().info(f"Opening gripper")
            self.ur_node.command_gripper(100, speed=255, force=1)
            self._publish_released()  # bewusstes Öffnen: nichts mehr gehalten
        else:
            self.get_logger().info(f"Closing gripper")
            self.ur_node.command_gripper(250, speed=255, force=255)
            self.check_tool_grasped()


    def check_tool_grasped(self):
        """Prüft nach dem Schließen über das Robotiq-Objekterkennungsregister,
        ob ein Werkzeug gegriffen wurde, und publiziert das Ergebnis auf
        /tool_grasped.

        Voraussetzung: Die Greifbewegung ist abgeschlossen (command_gripper
        schläft die berechnete Dauer) → gOBJ ist nicht mehr "in motion" (0).

        gOBJ == 2 -> beim Schließen auf Widerstand gestoppt = Objekt gegriffen
        gOBJ == 3 -> ohne Kontakt auf Zielposition = nichts / herausgefallen
        """
        # Erst warten, bis die Greifbewegung abgeschlossen ist (gOBJ != 0),
        # sonst liest man Werte aus der noch laufenden Bewegung.
        obj = self.ur_node.wait_for_gripper_stopped()
        pos = self.ur_node.query_gripper_var("POS")
        return self._evaluate_and_publish_grasp(obj, pos)


    def verify_grasp_callback(self, _msg):
        """On-Demand-Frischprüfung (z. B. direkt nach einem Lift). Der Greifer
        steht still, daher direkt gOBJ/gPO abfragen — ohne den Bewegungs-Settle
        von check_tool_grasped. Liefert einen frischen /tool_grasped-Wert, statt
        den evtl. veralteten Monitor-Wert abwarten zu müssen."""
        obj = self.ur_node.query_gripper_var("OBJ")
        pos = self.ur_node.query_gripper_var("POS")
        self.get_logger().info("On-demand Greif-Frischprüfung angefordert.")
        self._evaluate_and_publish_grasp(obj, pos)


    def _evaluate_and_publish_grasp(self, obj, pos):
        """Bewertet gOBJ/gPO, publiziert /tool_grasped und (de)aktiviert den
        Verlust-Monitor entsprechend."""
        grasped = (obj == 2)
        if self.grasp_use_pos_crosscheck and pos is not None:
            # Bei rausgefallenem Werkzeug fahren die Finger weiter zu (gPO nahe
            # Vollschluss). Mit Werkzeug stoppen sie davor.
            grasped = grasped and (pos < self.grasp_empty_close_pos - self.grasp_pos_margin)

        if obj is None:
            self.get_logger().warn("Werkzeug-Greif-Check: keine Antwort vom Gripper (OBJ=None).")

        self.get_logger().info(
            f"Werkzeug-Greif-Check: gOBJ={obj}, gPO={pos} -> grasped={grasped}"
        )

        msg = Bool()
        msg.data = bool(grasped)
        self.tool_grasped_publisher.publish(msg)

        # Bei erfolgreichem Griff die kontinuierliche Überwachung aktivieren,
        # damit ein späteres Herausziehen/Herausfallen erkannt wird.
        self.monitoring_active = bool(grasped)
        return grasped


    def _publish_released(self):
        """Greifer wurde absichtlich geöffnet (Handover-Release / normales
        Öffnen): Ground-Truth aktualisieren — nichts mehr gehalten — und den
        Verlust-Monitor deaktivieren. Sonst bliebe /tool_grasped auf 'true'
        hängen und der Holding-Guard würde künftige Picks fälschlich blockieren."""
        self.monitoring_active = False
        msg = Bool()
        msg.data = False
        self.tool_grasped_publisher.publish(msg)


    def _monitor_grasp(self):
        """Pollt gOBJ, solange ein Werkzeug gehalten wird. Kippt gOBJ von 2 (Objekt
        gehalten) auf 3 (leer durchgeschlossen), gilt das Werkzeug als verloren."""
        if not self.monitoring_active:
            return

        obj = self.ur_node.query_gripper_var("OBJ")
        if obj is None:
            return  # Keine Antwort -> nicht fälschlich Verlust melden

        if obj == 3:
            pos = self.ur_node.query_gripper_var("POS")
            self.get_logger().warn(
                f"Werkzeug verloren! gOBJ={obj}, gPO={pos}"
            )
            msg = Bool()
            msg.data = False
            self.tool_grasped_publisher.publish(msg)
            self.monitoring_active = False


    def gripper_zeroer_callback(self, bool_msg):
        # Extrahiere bool aus nachricht
        self.zeroer_active_ = bool_msg.data
        self.get_logger().info(f"Aktivitätsstatus: {'Aktiv' if self.zeroer_active_ else 'Inaktiv'}")
        if self.zeroer_active_:
            self.reset_force_offset()

    def gripper_position_callback(self, position_msg):
        position = int(position_msg.data)
        if not 0 <= position <= 255:
            self.get_logger().error(f"Invalid reclaim gripper position: {position}")
            return

        self.get_logger().info(f"Moving gripper to reclaim position {position}")
        self.ur_node.command_gripper(
            position,
            speed=self.reclaim_close_speed,
            force=self.reclaim_close_force,
        )

        # Nur bei einem Schließbefehl prüfen, ob ein Werkzeug gegriffen wurde.
        if position >= self.grasp_empty_close_pos - self.grasp_pos_margin:
            self.check_tool_grasped()
        elif position <= 110:
            self._publish_released()  # Öffnen über Positionsbefehl

        done_msg = Bool()
        done_msg.data = True
        self.gripper_position_done_publisher.publish(done_msg)
        self.get_logger().info("Published reclaim gripper position completion.")

    def force_callback(self, msg):
        if not self.zeroer_active_:
            return
        
        # Falls der Offset noch nicht gesetzt wurde, speichere ihn als Nullpunkt
        if self.force_offset is None:
            self.force_offset = msg.wrench.force
            self.torque_offset = msg.wrench.torque
            self.get_logger().info("🔹 Offset gespeichert: Setze aktuelle Werte als Null.")

        # Korrigierte Werte berechnen
        force_x = msg.wrench.force.x - self.force_offset.x
        force_y = msg.wrench.force.y - self.force_offset.y
        force_z = msg.wrench.force.z - self.force_offset.z

        # Nur wenn sich die Kraft von der Nullposition signifikant ändert, soll der Greifer öffnen
        if abs(force_x) > 2 or abs(force_y) > 2 or abs(force_z) > 2:
            self.get_logger().info("Force threshold exceeded, opening gripper.")
            self.ur_node.command_gripper(100, speed=255, force=1) # 0 = auf, 255 = ganz zu
            self._publish_released()  # Werkzeug übergeben: nichts mehr gehalten
            msg = Bool()
            msg.data = True
            self.status_publisher.publish(msg)
            self.get_logger().info("Gripper open.")
            self.zeroer_active_ = False


    def reset_force_offset(self):
        """Setzt die Kraft- und Drehmomentwerte auf 0 zurück."""
        self.force_offset = None
        self.torque_offset = None
        self.get_logger().info("Kraftsensor-Offset wurde zurückgesetzt!")

    def destroy_node(self):
        self.ur_node.close_connections()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SocketControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by user request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
