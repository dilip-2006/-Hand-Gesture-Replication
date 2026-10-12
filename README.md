# 5-Servo Robotic Hand: Arduino Uno + MediaPipe

## What this includes
- `arduino_hand.ino`: Arduino Uno servo receiver.
- `gesture_hand.py`: webcam + MediaPipe tracking, joint-angle mapping, smoothing, and serial commands.

## Hardware wiring
Servo signal pins:
- Thumb: D3
- Index: D5
- Middle: D6
- Ring: D9
- Little: D10

Power:
- Use a regulated external 5 V supply suitable for the servos' combined stall current.
- Connect external supply GND to Arduino GND.
- Do not power five servos from the Uno 5 V pin.
- Check that the mechanical hand cannot pinch or overtravel. Begin with the linkage unloaded.

## Software setup
Use Python 3.10 where possible, then:
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install opencv-python mediapipe pyserial
```

On Windows, activate with `.venv\\Scripts\\activate`.

## Run
1. Open `arduino_hand.ino` in Arduino IDE.
2. Select **Arduino Uno** and the correct port, then upload.
3. Close Arduino Serial Monitor (only one program can own the serial port).
4. Install Python dependencies.
5. Run Linux:
   ```bash
   python3 gesture_hand.py --port /dev/ttyACM0
   ```
   Common alternative Linux port: `/dev/ttyUSB0`.
   Windows example:
   ```bash
   python gesture_hand.py --port COM3
   ```
6. Press `q` or Escape to quit.

Test tracking without sending commands:
```bash
python3 gesture_hand.py --dry-run
```

## Serial protocol
Python sends newline-terminated commands at up to 20 Hz:
`A,90,90,90,90,90`

Order: thumb, index, middle, ring, little.
Arduino validates all five values, constrains them to the configured servo limits, then updates the servos.

## Calibration
In `gesture_hand.py`, edit:
- `OPEN_ANGLES`: servo angle for an extended finger.
- `CLOSED_ANGLES`: servo angle for a flexed finger.
- `STRAIGHT_DEG`: measured landmark joint angle when finger is extended.
- `BENT_DEG`: measured landmark joint angle when finger is flexed.

If a finger moves the wrong way, swap that finger's open and closed servo angles. If the linkage reaches its mechanical stop, reduce its range. Calibration depends on the printed hand and servo horn alignment.

In `arduino_hand.ino`, `SERVO_MIN` and `SERVO_MAX` are additional mechanical limits. Keep them consistent with the safe ranges of your particular mechanism.

## Notes / limitations
- This uses a 2D webcam projection. Finger joint angles are estimates and may be inaccurate when fingers overlap or are viewed edge-on.
- The thumb is especially sensitive to hand orientation; you may need to tune its angle thresholds.
- When the hand is not detected, the controller holds the last commanded position; it does not automatically open or close the hand.
- This assumes one servo per finger. A tendon-driven or multi-servo finger mechanism needs different mapping.
- Test at low speed / without load and keep hands clear of moving linkages.
