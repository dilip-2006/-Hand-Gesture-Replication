#!/usr/bin/env python3
"""
Real-time webcam gesture replication for a 5-servo robotic hand.

Requirements:
  pip install opencv-python mediapipe pyserial

Run:
  python3 gesture_hand.py --port /dev/ttyACM0
Windows example:
  python gesture_hand.py --port COM3

Serial protocol:
  A,<thumb>,<index>,<middle>,<ring>,<little>\\n
"""

import argparse
import math
import time
from collections import deque

import cv2
import mediapipe as mp
import serial
from serial import SerialException


# ---------------------------------------------------------------------
# CALIBRATION: edit these values to match your mechanism.
# Each list is in servo degrees, ordered:
# thumb, index, middle, ring, little.
#
# Start with conservative ranges. Use --dry-run first, then calibrate
# each finger individually with the mechanism disconnected or unloaded.
# If a finger moves in the wrong direction, swap its open/closed values.
# ---------------------------------------------------------------------
OPEN_ANGLES = [90, 90, 90, 90, 90]
CLOSED_ANGLES = [35, 25, 25, 25, 25]

# Flexion-angle ranges estimated from webcam landmarks.
# A finger joint angle near STRAIGHT_DEG means open;
# near BENT_DEG means closed. Tune per hand/camera if needed.
STRAIGHT_DEG = [155, 165, 165, 165, 165]
BENT_DEG     = [75,  95,  95,  95,  95]

# Smooth output: higher = smoother but slower.
SMOOTHING_ALPHA = 0.35
SEND_RATE_HZ = 20
CAMERA_INDEX = 0

# MediaPipe landmark indices:
# Thumb: angle at joint 3 using points 2,3,4.
# Other fingers: angle at PIP using MCP, PIP, DIP.
FINGER_TRIPLETS = [
    (2, 3, 4),       # thumb
    (5, 6, 7),       # index
    (9, 10, 11),     # middle
    (13, 14, 15),    # ring
    (17, 18, 19),    # little
]
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Little"]


def angle_degrees(a, b, c):
    """Angle ABC in degrees using normalized image-space landmarks."""
    bax = a.x - b.x
    bay = a.y - b.y
    bcx = c.x - b.x
    bcy = c.y - b.y

    denom = math.hypot(bax, bay) * math.hypot(bcx, bcy)
    if denom < 1e-8:
        return 180.0

    cosine = (bax * bcx + bay * bcy) / denom
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def clamp(value, low, high):
    return max(low, min(high, value))


def map_flexion_to_servo(angle, straight_deg, bent_deg, open_servo, closed_servo):
    """Map a tracked joint angle to a calibrated servo angle."""
    span = straight_deg - bent_deg
    if abs(span) < 1e-6:
        flex = 0.0
    else:
        flex = (straight_deg - angle) / span
    flex = clamp(flex, 0.0, 1.0)
    return int(round(open_servo + flex * (closed_servo - open_servo)))


def get_finger_angles(hand_landmarks):
    points = hand_landmarks.landmark
    result = []
    for i, (a_idx, b_idx, c_idx) in enumerate(FINGER_TRIPLETS):
        measured = angle_degrees(
            points[a_idx], points[b_idx], points[c_idx]
        )
        servo_angle = map_flexion_to_servo(
            measured,
            STRAIGHT_DEG[i],
            BENT_DEG[i],
            OPEN_ANGLES[i],
            CLOSED_ANGLES[i],
        )
        result.append((servo_angle, measured))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Arduino serial port, e.g. /dev/ttyACM0 or COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX)
    parser.add_argument("--dry-run", action="store_true",
                        help="Track and display angles without connecting to Arduino")
    args = parser.parse_args()

    arduino = None
    if not args.dry_run:
        try:
            arduino = serial.Serial(args.port, args.baud, timeout=0.05)
            # Uno resets when serial port opens; allow bootloader to finish.
            time.sleep(2.0)
            arduino.reset_input_buffer()
            print(f"Connected to Arduino on {args.port} at {args.baud} baud.")
        except SerialException as exc:
            raise SystemExit(
                f"Could not open Arduino port {args.port}: {exc}\n"
                "Check the port, USB cable, permissions, and close Arduino Serial Monitor."
            )

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        if arduino:
            arduino.close()
        raise SystemExit("Could not open webcam. Try --camera 1 or check camera permissions.")

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

    smoothed = [float(x) for x in OPEN_ANGLES]
    last_send = 0.0
    previous_time = time.time()
    fps = 0.0

    try:
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.65,
            min_tracking_confidence=0.60,
        ) as hands:
            while True:
                ok, frame = cap.read()
                if not ok:
                    print("Webcam frame unavailable.")
                    break

                # Mirror view so movement feels natural.
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                results = hands.process(rgb)
                rgb.flags.writeable = True

                now = time.time()
                dt = now - previous_time
                previous_time = now
                if dt > 0:
                    instant_fps = 1.0 / dt
                    fps = instant_fps if fps == 0 else 0.85 * fps + 0.15 * instant_fps

                if results.multi_hand_landmarks:
                    landmarks = results.multi_hand_landmarks[0]
                    mp_draw.draw_landmarks(
                        frame, landmarks, mp_hands.HAND_CONNECTIONS
                    )

                    measured = get_finger_angles(landmarks)
                    targets = [item[0] for item in measured]

                    # Exponential smoothing.
                    for i, target in enumerate(targets):
                        smoothed[i] += SMOOTHING_ALPHA * (target - smoothed[i])

                    if now - last_send >= 1.0 / SEND_RATE_HZ:
                        angles = [int(round(x)) for x in smoothed]
                        if arduino:
                            command = "A," + ",".join(map(str, angles)) + "\n"
                            arduino.write(command.encode("ascii"))
                        last_send = now

                    for i, (name, (servo_angle, joint_angle)) in enumerate(
                        zip(FINGER_NAMES, measured)
                    ):
                        y = 80 + i * 36
                        text = f"{name:6s} servo:{int(round(smoothed[i])):3d}  joint:{joint_angle:5.1f}"
                        cv2.putText(frame, text, (15, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                                    (30, 240, 140), 2, cv2.LINE_AA)
                    status = "TRACKING"
                    status_color = (30, 240, 140)
                else:
                    # Safety behavior: hold last servo target if hand disappears.
                    status = "NO HAND - HOLDING LAST POSITION"
                    status_color = (0, 180, 255)

                cv2.putText(frame, f"FPS: {fps:.1f}", (15, frame.shape[0] - 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 220, 30), 2)
                cv2.putText(frame, status, (15, frame.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

                cv2.imshow("Robotic Hand Gesture Control - press Q to quit", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        if arduino:
            try:
                arduino.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
