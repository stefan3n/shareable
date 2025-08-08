#include "HardwareConfig.h"

unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 500;

void setup() {
  Serial.begin(115200);
  // Set pinMode for new hardware config (2 motors)
  pinMode(MOTOR_PWM_RIGHT, OUTPUT);
  pinMode(MOTOR_ROTATION_RIGHT, OUTPUT);
  pinMode(MOTOR_PWM_LEFT, OUTPUT);
  pinMode(MOTOR_ROTATION_LEFT, OUTPUT);
  pinMode(MOTOR_BREAK, OUTPUT); // optional, pentru frana
  stopMotors();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('>');
    if (cmd.startsWith("<")) {
      cmd.remove(0, 1);
      if (cmd == "E") {
        stopMotors();
        Serial.println("<OK_STOP>");
      } else {
        int vals[4];
        int idx = 0;
        char *token = strtok((char*)cmd.c_str(), ",");
        while (token != NULL && idx < 4) {
          vals[idx++] = atoi(token);
          token = strtok(NULL, ",");
        }
        if (idx == 4) {
          // Dreapta
          digitalWrite(MOTOR_ROTATION_RIGHT, vals[0]);
          analogWrite(MOTOR_PWM_RIGHT, vals[1]);
          // Stanga
          digitalWrite(MOTOR_ROTATION_LEFT, vals[2]);
          analogWrite(MOTOR_PWM_LEFT, vals[3]);
          // Optional: dezactiveaza frana
          digitalWrite(MOTOR_BREAK, LOW);
          lastCmdTime = millis();
          Serial.print("<OK_SET,");
          Serial.print(vals[0]); Serial.print(",");
          Serial.print(vals[1]); Serial.print(",");
          Serial.print(vals[2]); Serial.print(",");
          Serial.print(vals[3]); Serial.println(">");
        } else {
          Serial.println("<ERR_FORMAT>");
        }
      }
    }
  }
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    stopMotors();
  }
}

void stopMotors() {
  analogWrite(MOTOR_PWM_RIGHT, 0);
  analogWrite(MOTOR_PWM_LEFT, 0);
  // Activeaza frana daca e nevoie
  digitalWrite(MOTOR_BREAK, HIGH);
}