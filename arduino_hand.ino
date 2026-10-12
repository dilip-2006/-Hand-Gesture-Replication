#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// Arduino Uno: one servo per finger.
// Serial command format: A,<thumb>,<index>,<middle>,<ring>,<little>\n
const byte NUM_FINGERS = 5;
const byte SERVO_PINS[NUM_FINGERS] = {3, 5, 6, 9, 10};

// Mechanical calibration limits. Adjust these to prevent over-travel.
// These are safe starting values, NOT universal values for every hand.
const int SERVO_MIN[NUM_FINGERS] = {10, 10, 10, 10, 10};
const int SERVO_MAX[NUM_FINGERS] = {170, 170, 170, 170, 170};

Servo fingerServo[NUM_FINGERS];
char lineBuffer[80];
byte lineLength = 0;

void setup() {
  Serial.begin(115200);

  for (byte i = 0; i < NUM_FINGERS; i++) {
    fingerServo[i].attach(SERVO_PINS[i]);
    fingerServo[i].write(90);  // neutral position on startup
  }

  Serial.println(F("READY"));
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      processLine(lineBuffer);
      lineLength = 0;
    } else if (c != '\r') {
      if (lineLength < sizeof(lineBuffer) - 1) {
        lineBuffer[lineLength++] = c;
      } else {
        lineLength = 0;  // discard overlong command
        Serial.println(F("ERR:LINE_TOO_LONG"));
      }
    }
  }
}

void processLine(char *line) {
  // Expected: A,90,90,90,90,90
  if (line[0] != 'A' || line[1] != ',') {
    Serial.println(F("ERR:BAD_PREFIX"));
    return;
  }

  int values[NUM_FINGERS];
  char *token = strtok(line + 2, ",");

  for (byte i = 0; i < NUM_FINGERS; i++) {
    if (token == NULL || *token == '\0') {
      Serial.println(F("ERR:NEED_5_ANGLES"));
      return;
    }

    char *endPtr;
    long value = strtol(token, &endPtr, 10);
    if (*endPtr != '\0' || value < 0 || value > 180) {
      Serial.println(F("ERR:INVALID_ANGLE"));
      return;
    }

    values[i] = (int)value;
    token = strtok(NULL, ",");
  }

  if (token != NULL) {
    Serial.println(F("ERR:TOO_MANY_ANGLES"));
    return;
  }

  for (byte i = 0; i < NUM_FINGERS; i++) {
    int angle = constrain(values[i], SERVO_MIN[i], SERVO_MAX[i]);
    fingerServo[i].write(angle);
  }

  Serial.println(F("OK"));
}
