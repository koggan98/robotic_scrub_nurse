import cv2
import mediapipe as mp
import time
import math

class MediaPipeTracker:
    def __init__(self, static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5):
        """Initialisiert MediaPipe Hands."""
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def process_frame(self, frame):
        """Verarbeitet ein Frame und gibt die Handlandmarken zurück."""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        return results

    def draw_landmarks(self, frame, hand_landmarks):
        """Zeichnet die Handlandmarken auf das Frame."""
        if hand_landmarks:
            for landmarks in hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, landmarks, self.mp_hands.HAND_CONNECTIONS)
        return frame

class HandGestureTracker:
    def __init__(self):
        self.last_state = None  # Zuletzt erkannter Zustand ('open' oder 'closed')
        self.transition_times = []  # Zeiten, zu denen Zustandsänderungen erkannt wurden

    def is_hand_open(self, hand_landmarks, mp_hands):
        """Überprüft, ob die Hand offen ist anhand des x wertes der fingerspitzen zur fingerbasis (waagerechte im bild)."""
        fingers_open = []
        for finger in [mp_hands.HandLandmark.THUMB_TIP, mp_hands.HandLandmark.INDEX_FINGER_TIP,
                       mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_TIP,
                       mp_hands.HandLandmark.PINKY_TIP]:
            fingertip = hand_landmarks.landmark[finger]
            finger_base = hand_landmarks.landmark[finger - 2]
            fingers_open.append(fingertip.x < finger_base.x)
        return sum(fingers_open) >= 4

    def detect_double_open_close(self, hand_landmarks, mp_hands):
        """Erkennt, ob die Hand zwei Mal nacheinander geschlossen und geöffnet wurde."""
        current_state = "open" if self.is_hand_open(hand_landmarks, mp_hands) else "closed"
        current_time = time.time()

        if current_state != self.last_state:
            self.transition_times.append(current_time)
            self.last_state = current_state

        self.transition_times = [t for t in self.transition_times if current_time - t <= 1]

        if len(self.transition_times) >= 4:
            self.transition_times = []
            return True
        return False

    def _angle_deg(self, a, b, c):
        """Berechnet den Winkel (in Grad) im Punkt b zwischen a-b-c."""
        ba = (a.x - b.x, a.y - b.y, a.z - b.z)
        bc = (c.x - b.x, c.y - b.y, c.z - b.z)
        dot = ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]
        norm_ba = math.sqrt(ba[0] ** 2 + ba[1] ** 2 + ba[2] ** 2)
        norm_bc = math.sqrt(bc[0] ** 2 + bc[1] ** 2 + bc[2] ** 2)
        denom = norm_ba * norm_bc
        if denom < 1e-6:
            return 0.0
        cos_angle = max(-1.0, min(1.0, dot / denom))
        return math.degrees(math.acos(cos_angle))

    def is_finger_extended(self, hand_landmarks, mp_hands, finger, angle_threshold_deg):
        if finger == "thumb":
            tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
            mcp = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]
            angle = self._angle_deg(tip, ip, mcp)
        elif finger == "index":
            tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
            mcp = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
            angle = self._angle_deg(tip, pip, mcp)
        elif finger == "middle":
            tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
            mcp = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
            angle = self._angle_deg(tip, pip, mcp)
        elif finger == "ring":
            tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
            pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
            mcp = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP]
            angle = self._angle_deg(tip, pip, mcp)
        elif finger == "pinky":
            tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
            pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]
            mcp = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
            angle = self._angle_deg(tip, pip, mcp)
        else:
            return False
        return angle >= angle_threshold_deg

    def is_shaka(self, hand_landmarks, mp_hands, angle_threshold_deg):
        thumb = self.is_finger_extended(hand_landmarks, mp_hands, "thumb", angle_threshold_deg)
        pinky = self.is_finger_extended(hand_landmarks, mp_hands, "pinky", angle_threshold_deg)
        index = self.is_finger_extended(hand_landmarks, mp_hands, "index", angle_threshold_deg)
        middle = self.is_finger_extended(hand_landmarks, mp_hands, "middle", angle_threshold_deg)
        ring = self.is_finger_extended(hand_landmarks, mp_hands, "ring", angle_threshold_deg)
        return thumb and pinky and (not index) and (not middle) and (not ring)

    def shaka_score(self, hand_landmarks, mp_hands, angle_threshold_deg):
        """Score zur Auswahl bei mehreren Shaka-Haenden."""
        thumb_ext = self.is_finger_extended(hand_landmarks, mp_hands, "thumb", angle_threshold_deg)
        pinky_ext = self.is_finger_extended(hand_landmarks, mp_hands, "pinky", angle_threshold_deg)
        index_ext = self.is_finger_extended(hand_landmarks, mp_hands, "index", angle_threshold_deg)
        middle_ext = self.is_finger_extended(hand_landmarks, mp_hands, "middle", angle_threshold_deg)
        ring_ext = self.is_finger_extended(hand_landmarks, mp_hands, "ring", angle_threshold_deg)
        score = 0.0
        score += 2.0 if thumb_ext else 0.0
        score += 2.0 if pinky_ext else 0.0
        score -= 1.0 if index_ext else 0.0
        score -= 1.0 if middle_ext else 0.0
        score -= 1.0 if ring_ext else 0.0
        return score
   
    def get_hand_center_wrist(self, hand_landmarks, frame_shape, mp_hands):
        """Berechnet die Mitte der Handfläche basierend nur auf stabilen Landmarken."""
        wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
        middle_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

        # Durchschnitt aus stabilen Punkten
        x = int((wrist.x + middle_mcp.x) / 2 * frame_shape[1])
        y = int((wrist.y + middle_mcp.y) / 2 * frame_shape[0])
        return x, y
