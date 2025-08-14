#include <Servo.h>

const int ENASTEPPER = 3;
const int DIR_STEPPER = 13;
const int PUL_STEPPER = 2;

const int ENA1 = 4;
const int IN1_1 = 5;
const int IN2_1 = 6;

const int ENA2 = 4;  
const int IN1_2 = 7;
const int IN2_2 = 8;

const int ENA3 = 12;
const int IN1_3 = 9;
const int IN2_3 = 10;

const int CLAW_PIN = 11;
Servo clawServo;
int clawAngle = 90; // Initial position
Servo rotateClawServo;
int rotateClawAngle = 90; // Initial position
#define ROTATE_CLAW_PIN A5

#define FEEDBACK_PIN1 A2
#define FEEDBACK_PIN2 A0
#define FEEDBACK_PIN3 A1

#define ERROR_MARGIN 10


// ------------------ CLASA: CLAW ------------------------
void opencloseClaw(int direction) {
    // direction: -1 = stanga, 1 = dreapta
    clawAngle = constrain(clawAngle + direction * 10, 0, 180);
    clawServo.write(clawAngle);
}

// ------------------ CLASA: ROTIRE GHEARA ------------------------
void rotateClaw(int direction) {
    // direction: -1 = stanga, 1 = dreapta
    rotateClawAngle = constrain(rotateClawAngle + direction * 10, 0, 180);
    rotateClawServo.write(rotateClawAngle);
}

// ------------------ CLASA: STEPPER ------------------------
void moveStepperSteps(int direction, int steps) {
    digitalWrite(ENASTEPPER, HIGH); // Enable stepper
    digitalWrite(DIR_STEPPER, direction ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        digitalWrite(PUL_STEPPER, HIGH);
        delayMicroseconds(1000); // pulse width - mai lung pentru stabilitate
        digitalWrite(PUL_STEPPER, LOW);
        delayMicroseconds(1000); // pulse interval - mai lung pentru a reduce zgomotul
    }
    digitalWrite(ENASTEPPER, LOW); // Disable stepper after move
}

// ------------------ CLASA: ACTUATOR ------------------------
void stopActuator(int ENA, int IN1, int IN2) {
    digitalWrite(ENA, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}

void moveActuator(int ENA, int IN1, int IN2, int dir) {
    digitalWrite(ENA, HIGH);
    digitalWrite(IN1, dir == 0 ? HIGH : LOW);
    digitalWrite(IN2, dir == 0 ? LOW : HIGH);
}

int getFeedback(int FEEDBACK_PIN) {
    int suma = 0;
    const int nr_masuratori = 20;
    for (int i = 0; i < nr_masuratori; i++) {
        suma += analogRead(FEEDBACK_PIN);
        delay(2);
    }
    return suma / nr_masuratori;
}

void moveActuatorGivenPosition(int ENA, int IN1, int IN2, int targetPosition, int FEEDBACK_PIN) {
    int currentPosition = getFeedback(FEEDBACK_PIN);
    if (abs(currentPosition - targetPosition) <= ERROR_MARGIN) {
        stopActuator(ENA, IN1, IN2);
        return;
    }
    int direction = (currentPosition < targetPosition) ? 0 : 1;
    moveActuator(ENA, IN1, IN2, direction);
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) {  // timeout 3 secunde
        currentPosition = getFeedback(FEEDBACK_PIN);
        if (abs(currentPosition - targetPosition) <= ERROR_MARGIN) {
            break;
        }
        delay(10);
    }
    stopActuator(ENA, IN1, IN2);
}

void moveAllActuators(int target1, int target2, int target3, int finalPos[3]) {
    // Move all three actuators to their target positions
    Serial.print("Moving actuators to positions: ");
    Serial.print(target1);
    Serial.print(", ");
    Serial.print(target2);
    Serial.print(", ");
    Serial.println(target3);
    
    // Move actuator 1
    moveActuatorGivenPosition(ENA1, IN1_1, IN2_1, target1, FEEDBACK_PIN1);
    finalPos[0] = getFeedback(FEEDBACK_PIN1);
    
    // Move actuator 2
    moveActuatorGivenPosition(ENA2, IN1_2, IN2_2, target2, FEEDBACK_PIN2);
    finalPos[1] = getFeedback(FEEDBACK_PIN2);
    
    // Move actuator 3
    moveActuatorGivenPosition(ENA3, IN1_3, IN2_3, target3, FEEDBACK_PIN3);
    finalPos[2] = getFeedback(FEEDBACK_PIN3);
    
    Serial.print("Final positions: ");
    Serial.print(finalPos[0]);
    Serial.print(", ");
    Serial.print(finalPos[1]);
    Serial.print(", ");
    Serial.println(finalPos[2]);
}

// ------------------ CLASA: PARSARE COMENZI ------------------------
void parseAndExecuteCommand(int actuator, int target, int &finalPos) {
    switch (actuator) {
        case 1:
            moveActuatorGivenPosition(ENA1, IN1_1, IN2_1, target, FEEDBACK_PIN1);
            finalPos = getFeedback(FEEDBACK_PIN1);
            break;
        case 2:
            moveActuatorGivenPosition(ENA2, IN1_2, IN2_2, target, FEEDBACK_PIN2);
            finalPos = getFeedback(FEEDBACK_PIN2);
            break;
        case 3:
            moveActuatorGivenPosition(ENA3, IN1_3, IN2_3, target, FEEDBACK_PIN3);
            finalPos = getFeedback(FEEDBACK_PIN3);
            break;
        case 11: // moveOneUp
        {
            Serial.println("HELLO!");
            int current = getFeedback(FEEDBACK_PIN1);
            int targetUp = current + 20;
            moveActuatorGivenPosition(ENA1, IN1_1, IN2_1, targetUp, FEEDBACK_PIN1);
            finalPos = getFeedback(FEEDBACK_PIN1);
            break;
        }
        case 12: // moveOneDown
        {
            int current = getFeedback(FEEDBACK_PIN1);
            int targetDown = current - 20;
            moveActuatorGivenPosition(ENA1, IN1_1, IN2_1, targetDown, FEEDBACK_PIN1);
            finalPos = getFeedback(FEEDBACK_PIN1);
            break;
        }
        case 21: // moveTwoUp
        {
            int current = getFeedback(FEEDBACK_PIN2);
            int targetUp = current + 20;
            moveActuatorGivenPosition(ENA2, IN1_2, IN2_2, targetUp, FEEDBACK_PIN2);
            finalPos = getFeedback(FEEDBACK_PIN2);
            break;
        }
        case 22: // moveTwoDown
        {
            int current = getFeedback(FEEDBACK_PIN2);
            int targetDown = current - 20;
            moveActuatorGivenPosition(ENA2, IN1_2, IN2_2, targetDown, FEEDBACK_PIN2);
            finalPos = getFeedback(FEEDBACK_PIN2);
            break;
        }
        case 31: // moveThreeUp
        {
            int current = getFeedback(FEEDBACK_PIN3);
            int targetUp = current + 20;
            moveActuatorGivenPosition(ENA3, IN1_3, IN2_3, targetUp, FEEDBACK_PIN3);
            finalPos = getFeedback(FEEDBACK_PIN3);
            break;
        }
        case 32: // moveThreeDown
        {
            int current = getFeedback(FEEDBACK_PIN3);
            int targetDown = current - 20;
            moveActuatorGivenPosition(ENA3, IN1_3, IN2_3, targetDown, FEEDBACK_PIN3);
            finalPos = getFeedback(FEEDBACK_PIN3);
            break;
        }
        case 4:
            if (target == 0) {
                opencloseClaw(-1); // stanga
                finalPos = clawAngle;
            } else if (target == 1) {
                opencloseClaw(1); // dreapta
                finalPos = clawAngle;
            } else {
                Serial.println("Unknown rotate command!");
                finalPos = -1;
            }
            break;
        case 5:
            if (target == 0) {
                rotateClaw(-1); // stanga
                finalPos = rotateClawAngle;
            } else if (target == 1) {
                rotateClaw(1); // dreapta
                finalPos = rotateClawAngle;
            } else {
                Serial.println("Unknown rotate command!");
                finalPos = -1;
            }
            break;
        case 6:
            // target: 0 = stanga, 1 = dreapta
            if (target == 0) {
                moveStepperSteps(1, 50); 
                finalPos = 0;
            } else if (target == 1) {
                moveStepperSteps(0, 50); 
                finalPos = 1;
            } else {
                Serial.println("Unknown stepper command!");
                finalPos = -1;
            }
            break;
        case 123:
            // Move all 3 actuators to specific positions (go command)
            // target parameter is not used here, positions are parsed separately
            finalPos = 123; // Special return value for go command
            break;
        default:
            Serial.println("Unknown actuator!");
            finalPos = -1;
            break;
    }
}

// ------------------ SETUP + LOOP ------------------------


void setupPins() {
    pinMode(FEEDBACK_PIN1, INPUT);
    pinMode(FEEDBACK_PIN2, INPUT);
    pinMode(FEEDBACK_PIN3, INPUT);
    pinMode(ENA1, OUTPUT);
    pinMode(ENA2, OUTPUT);
    pinMode(ENA3, OUTPUT);
    pinMode(ENASTEPPER, OUTPUT);
    pinMode(DIR_STEPPER, OUTPUT);
    pinMode(PUL_STEPPER, OUTPUT);
    pinMode(IN1_1, OUTPUT);
    pinMode(IN2_1, OUTPUT);
    pinMode(IN1_2, OUTPUT);
    pinMode(IN2_2, OUTPUT);
    pinMode(IN1_3, OUTPUT);
    pinMode(IN2_3, OUTPUT);
    digitalWrite(ENASTEPPER, LOW);
}

void setupServos() {
    clawServo.attach(CLAW_PIN);
    rotateClawServo.attach(ROTATE_CLAW_PIN);
}

void setup() {
    Serial.begin(115200);
    setupPins();
    setupServos();
    Serial.println("Arm arduino is ready!");
}

void sendFeedback(int actuator, int finalPos) {
    Serial.print("<");
    Serial.print(actuator);
    Serial.print(",");
    Serial.print(finalPos);
    Serial.println(">");
}

void processSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('>'); // read until '>'
        int startIdx = input.indexOf('<');
        if (startIdx != -1) {
            String content = input.substring(startIdx + 1); // exclude '<'
            content.trim();
            
            // Check if it's a "go" command (format: <123, value1,value2,value3>)
            if (content.startsWith("123,")) {
                // Parse the three potentiometer values
                String values = content.substring(4); // Remove "123,"
                values.trim();
                
                int firstComma = values.indexOf(',');
                int secondComma = values.indexOf(',', firstComma + 1);
                
                if (firstComma != -1 && secondComma != -1) {
                    String value1Str = values.substring(0, firstComma);
                    String value2Str = values.substring(firstComma + 1, secondComma);
                    String value3Str = values.substring(secondComma + 1);
                    
                    value1Str.trim();
                    value2Str.trim();
                    value3Str.trim();
                    
                    int target1 = value1Str.toInt();
                    int target2 = value2Str.toInt();
                    int target3 = value3Str.toInt();
                    
                    int finalPos[3];
                    moveAllActuators(target1, target2, target3, finalPos);
                    
                    // Send feedback for all three actuators
                    Serial.print("<123,");
                    Serial.print(finalPos[0]);
                    Serial.print(",");
                    Serial.print(finalPos[1]);
                    Serial.print(",");
                    Serial.print(finalPos[2]);
                    Serial.println(">");
                } else {
                    Serial.println("Invalid go command format!");
                }
            } else {
                // Accepta atat <actuator,target> cat si <actuator>
                int commaIdx = content.indexOf(',');
                int actuator = 0;
                int target = 0;
                bool valid = false;
                if (commaIdx != -1) {
                    // Format <actuator,target>
                    String actuatorStr = content.substring(0, commaIdx);
                    String valueStr = content.substring(commaIdx + 1);
                    actuatorStr.trim();
                    valueStr.trim();
                    actuator = actuatorStr.toInt();
                    target = valueStr.toInt();
                    valid = true;
                } else {
                    // Format <actuator> (fara target)
                    String actuatorStr = content;
                    actuatorStr.trim();
                    actuator = actuatorStr.toInt();
                    // Pentru comenzile speciale (11,12,21,22,31,32) target nu conteaza
                    if (actuator == 11 || actuator == 12 || actuator == 21 || actuator == 22 || actuator == 31 || actuator == 32) {
                        target = 0;
                        valid = true;
                    }
                }
                if (valid) {
                    int finalPos = 0;
                    parseAndExecuteCommand(actuator, target, finalPos);
                    if ((actuator >= 1 && actuator <= 6) || actuator == 11 || actuator == 12 || actuator == 21 || actuator == 22 || actuator == 31 || actuator == 32) {
                        sendFeedback(actuator, finalPos);
                    }
                }
            }
        }
    }
}

void loop() {
    processSerialInput();
    delay(20); // Debounce delay
}
