#include <WiFiS3.h>
#include "webpage.h"
#include "led.h"
#include "Arduino_LED_Matrix.h"
#include <Servo.h>

const char* ssid = "excavator";
const char* password = "password";

WiFiServer server(80);
ArduinoLEDMatrix matrix;

// ===== SPOON SERVO =====
#define SPOON_SERVO_PIN 13
const int SPOON_OPEN_POS = 80;   // degrees for open spoon (adjust as needed)
const int SPOON_CLOSED_POS = 10; // degrees for closed spoon (adjust as needed)
Servo spoonServo;

// ===== PWM RAMP SETTINGS =====
const unsigned long PWM_RAMP_TIME = 600;  // ms to reach 100%
const int MAX_PWM = 255;  // Maximum PWM value

// ===== MOTOR CONFIGURATION =====
struct MotorConfig {
  int pinForward;   // IN1/IN3/IN5/IN7/IN9
  int pinBackward;  // IN2/IN4/IN6/IN8/IN10
  int pinPWM;       // ENA/ENB/ENC/END/ENE
};

struct MotorState {
  unsigned long startTime;
  int targetSpeed;  // 0-255
  int currentSpeed; // 0-255
  bool isActive;
};

// Motor pin configurations
const MotorConfig MOTOR_CONFIGS[5] = {
  {2, 4, 3},   // Motor 1 (levý pás) - IN1, IN2, ENA
  {5, 6, 3},   // Motor 2 (pravý pás) - IN3, IN4, ENB
  {7, 8, 3},   // Motor 3 (lano - rope) - IN5, IN6, ENC
  {9, 10, 3},  // Motor 4 (hřeben - comb) - IN7, IN8, END
  {11, 12, 3}  // Motor 5 (otáčení - turn) - IN9, IN10, ENE
};

// Motor states
MotorState motors[5] = {
  {0, 0, 0, false},  // Motor 1
  {0, 0, 0, false},  // Motor 2
  {0, 0, 0, false},  // Motor 3 (rope)
  {0, 0, 0, false},  // Motor 4 (comb)
  {0, 0, 0, false}   // Motor 5 (turn)
};

// ===== WATCHDOG =====
unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 400; // ms

// ===== ANALOG INPUTS =====
int horizontalAngleA0 = 0;  // Pin A0 - horizontal
int verticalAngleA1 = 0;  // Pin A1 - vertical

void readAnalogInputs() {
  horizontalAngleA0 = analogRead(A0);
  verticalAngleA1 = analogRead(A1);
}

void emergencyStop() {
  // STOP ALL MOTORS IMMEDIATELY
  for (int i = 0; i < 5; i++) {
    const MotorConfig& config = MOTOR_CONFIGS[i];
    digitalWrite(config.pinForward, LOW);
    digitalWrite(config.pinBackward, LOW);
    analogWrite(config.pinPWM, 0);
    
    motors[i].isActive = false;
    motors[i].currentSpeed = 0;
  }
  
  Serial.println("!!! WATCHDOG STOP !!!");
}

void watchdogCheck() {
  if (millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
    emergencyStop();
    lastCmdTime = millis();
  }
}

// ===== PWM RAMPING =====
void updateMotorRamp() {
  unsigned long now = millis();

  for (int i = 0; i < 5; i++) {
    MotorState& motor = motors[i];
    if (motor.isActive) {
      unsigned long elapsed = now - motor.startTime;
      if (elapsed >= PWM_RAMP_TIME) {
        motor.currentSpeed = motor.targetSpeed;
      } else {
        motor.currentSpeed = (motor.targetSpeed * elapsed) / PWM_RAMP_TIME;
      }
      analogWrite(MOTOR_CONFIGS[i].pinPWM, motor.currentSpeed);
    }
  }
}

// ===== MOTOR CONTROL FUNCTIONS =====
void setMotorDirection(int motorNum, int direction) {
  // direction: 1=forward, -1=backward, 0=stop
  // motorNum: 0-4 (0-indexed)
  if (motorNum < 0 || motorNum >= 5) return;
  
  const MotorConfig& config = MOTOR_CONFIGS[motorNum];
  
  if (direction == 1) {
    digitalWrite(config.pinForward, HIGH);
    digitalWrite(config.pinBackward, LOW);
  } else if (direction == -1) {
    digitalWrite(config.pinForward, LOW);
    digitalWrite(config.pinBackward, HIGH);
  } else {
    digitalWrite(config.pinForward, LOW);
    digitalWrite(config.pinBackward, LOW);
  }
}

void startMotorRamp(int motorNum, int direction, int speed) {
  // direction: 1=forward, -1=backward
  // motorNum: 1-5 (1-indexed for compatibility)
  int idx = motorNum - 1;  // Convert to 0-indexed
  
  if (idx < 0 || idx >= 5) return;
  
  setMotorDirection(idx, direction);
  
  MotorState& motor = motors[idx];
  if (!motor.isActive) {
    motor.targetSpeed = speed;
    motor.startTime = millis();
    motor.isActive = true;
    motor.currentSpeed = 0;
  }
}

// ===== HELPER FUNCTION FOR LED MATRIX =====
void displayBitmap(uint8_t bitmap[][MAX_X]) {
  matrix.clear();
  matrix.renderBitmap(bitmap, MAX_Y, MAX_X);
}

// ===== MOVEMENT FUNCTIONS =====
void moveForward() {
  startMotorRamp(1, 1, MAX_PWM);  // Motor 1: forward, 100%
  startMotorRamp(2, 1, MAX_PWM);  // Motor 2: forward, 100%
  displayBitmap(FORWARD);
}

void moveBackward() {
  startMotorRamp(1, -1, MAX_PWM);  // Motor 1: backward, 100%
  startMotorRamp(2, -1, MAX_PWM);  // Motor 2: backward, 100%
  displayBitmap(BACKWARD);
}

void moveLeft() {
  startMotorRamp(1, 1, MAX_PWM);  // Motor 1 (levý): backward, 100%
  startMotorRamp(2, -1, MAX_PWM);   // Motor 2 (pravý): forward, 100%
  displayBitmap(LEFT);
}

void moveRight() {
  startMotorRamp(1, -1, MAX_PWM);   // Motor 1 (levý): forward, 100%
  startMotorRamp(2, 1, MAX_PWM);  // Motor 2 (pravý): backward, 100%
  displayBitmap(RIGHT);
}

void moveRopeOut() {
  startMotorRamp(3, 1, MAX_PWM);  // Motor 3: extend rope, 100%
  displayBitmap(ROPE_OUT);
}

void moveRopeIn() {
  startMotorRamp(3, -1, MAX_PWM);  // Motor 3: retract rope, 100%
  displayBitmap(ROPE_IN);
}

void moveCombOut() {
  startMotorRamp(4, 1, MAX_PWM);  // Motor 4: extend comb, 100%
  displayBitmap(COMB_OUT);
}

void moveCombIn() {
  startMotorRamp(4, -1, MAX_PWM);  // Motor 4: retract comb, 100%
  displayBitmap(COMB_IN);
}

void turnLeft() {
  startMotorRamp(5, -1, MAX_PWM);  // Motor 5: turn left, 100%
  displayBitmap(TURN_LEFT);
}

void turnRight() {
  startMotorRamp(5, 1, MAX_PWM);   // Motor 5: turn right, 100%
  displayBitmap(TURN_RIGHT);
}

void spoon_function(int open) {
  if (open == 1) {
    spoonServo.write(SPOON_OPEN_POS);
    displayBitmap(SO);
  } else {
    spoonServo.write(SPOON_CLOSED_POS);
    displayBitmap(SC);
  }
}

void L1_function() {
  displayBitmap(L1);
}

void L2_function() {
  displayBitmap(L2);
}

void L3_function() {
  displayBitmap(L3);
}

void processCommand(const String& cmd) {
  Serial.println(cmd);

  if (cmd == "stop") {
    matrix.clear();
    emergencyStop();
  }
  else if (cmd == "rope_out") { moveRopeOut(); }
  else if (cmd == "rope_in")  { moveRopeIn(); }
  else if (cmd == "comb_out") { moveCombOut(); }
  else if (cmd == "comb_in")  { moveCombIn(); }
  else if (cmd == "turn_left")  { turnLeft(); }
  else if (cmd == "turn_right") { turnRight(); }
  else if (cmd == "forward") { moveForward(); }
  else if (cmd == "back")    { moveBackward(); }
  else if (cmd == "left")    { moveLeft(); }
  else if (cmd == "right")   { moveRight(); }
  else if (cmd == "spoon_open")  { spoon_function(1); }
  else if (cmd == "spoon_close") { spoon_function(0); }
  else if (cmd == "L1") { L1_function(); }
  else if (cmd == "L2") { L2_function(); }
  else if (cmd == "L3") { L3_function(); }
}

void handleCommand(const String& req) {
  int idx = req.indexOf("c=");
  if (idx < 0) return;

  String cmd = req.substring(idx + 2);
  cmd = cmd.substring(0, cmd.indexOf(' '));

  lastCmdTime = millis();  // ⏱ watchdog reset

  processCommand(cmd);
}

void sendPage(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  client.println(webpage);
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\r');
  client.flush();

  if (req.indexOf("GET / ") >= 0) {
    sendPage(client);
  }
  else if (req.indexOf("GET /cmd") >= 0) {
    handleCommand(req);
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");
    client.println();
  }

  client.stop();
}

void setup() {
  matrix.begin();
  matrix.clear();

  Serial.begin(115200);

  // Initialize all motor pins
  for (int i = 0; i < 5; i++) {
    const MotorConfig& config = MOTOR_CONFIGS[i];
    pinMode(config.pinForward, OUTPUT);
    pinMode(config.pinBackward, OUTPUT);
    pinMode(config.pinPWM, OUTPUT);
  }

  // Initialize spoon servo
  spoonServo.attach(SPOON_SERVO_PIN);
  spoonServo.write(SPOON_CLOSED_POS); // start closed

  WiFi.beginAP(ssid, password);
  delay(2000);

  Serial.print("AP IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  handleClient();
  watchdogCheck();
  updateMotorRamp();  // Continuously update motor PWM ramps
  readAnalogInputs();  // Read A0 and A1
}
