#include <WiFiS3.h>
#include "webpage.h"

const char* ssid = "excavator";
const char* password = "password";

WiFiServer server(80);

// ===== L298N MOTOR CONTROL PINS =====
// Motor 1 (levý pás)
#define IN1 2  // Motor direction pin 1
#define IN2 3  // Motor direction pin 2
#define ENA 5  // PWM pin for motor 1

// Motor 2 (pravý pás)
#define IN3 4  // Motor direction pin 3
#define IN4 6  // Motor direction pin 4
#define ENB 9  // PWM pin for motor 2

// ===== PWM RAMP SETTINGS =====
const unsigned long PWM_RAMP_TIME = 2000;  // 2 seconds to reach 100%

// ===== MOTOR STATE =====
struct MotorState {
  unsigned long startTime;
  int targetSpeed;  // 0-255
  int currentSpeed; // 0-255
  bool isActive;
};

MotorState motor1 = {0, 0, 0, false};
MotorState motor2 = {0, 0, 0, false};

// ===== WATCHDOG =====
unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 300; // ms

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  WiFi.beginAP(ssid, password);
  delay(2000);

  Serial.print("AP IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void emergencyStop() {
  // STOP ALL MOTORS IMMEDIATELY
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, 0);
  
  motor1.isActive = false;
  motor2.isActive = false;
  motor1.currentSpeed = 0;
  motor2.currentSpeed = 0;
  
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

  // Update motor 1
  if (motor1.isActive) {
    unsigned long elapsed = now - motor1.startTime;
    if (elapsed >= PWM_RAMP_TIME) {
      motor1.currentSpeed = motor1.targetSpeed;
    } else {
      // Linear ramp: 0 → targetSpeed over PWM_RAMP_TIME ms
      motor1.currentSpeed = (motor1.targetSpeed * elapsed) / PWM_RAMP_TIME;
    }
    analogWrite(ENA, motor1.currentSpeed);
  }

  // Update motor 2
  if (motor2.isActive) {
    unsigned long elapsed = now - motor2.startTime;
    if (elapsed >= PWM_RAMP_TIME) {
      motor2.currentSpeed = motor2.targetSpeed;
    } else {
      // Linear ramp: 0 → targetSpeed over PWM_RAMP_TIME ms
      motor2.currentSpeed = (motor2.targetSpeed * elapsed) / PWM_RAMP_TIME;
    }
    analogWrite(ENB, motor2.currentSpeed);
  }
}

// ===== MOTOR CONTROL FUNCTIONS =====
void setMotorDirection(int motorNum, int direction) {
  // direction: 1=forward, -1=backward, 0=stop
  if (motorNum == 1) {
    if (direction == 1) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (direction == -1) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  } else if (motorNum == 2) {
    if (direction == 1) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (direction == -1) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}

void startMotorRamp(int motorNum, int direction, int speed) {
  // direction: 1=forward, -1=backward
  setMotorDirection(motorNum, direction);
  
  if (motorNum == 1) {
    motor1.targetSpeed = speed;
    motor1.startTime = millis();
    motor1.isActive = true;
    motor1.currentSpeed = 0;
  } else if (motorNum == 2) {
    motor2.targetSpeed = speed;
    motor2.startTime = millis();
    motor2.isActive = true;
    motor2.currentSpeed = 0;
  }
}

void moveForward() {
  Serial.println("Move FORWARD");
  startMotorRamp(1, 1, 255);  // Motor 1: forward, 100%
  startMotorRamp(2, 1, 255);  // Motor 2: forward, 100%
}

void moveBackward() {
  Serial.println("Move BACK");
  startMotorRamp(1, -1, 255);  // Motor 1: backward, 100%
  startMotorRamp(2, -1, 255);  // Motor 2: backward, 100%
}

void moveLeft() {
  Serial.println("Move LEFT");
  startMotorRamp(1, -1, 255);  // Motor 1 (levý): backward, 100%
  startMotorRamp(2, 1, 255);   // Motor 2 (pravý): forward, 100%
}

void moveRight() {
  Serial.println("Move RIGHT");
  startMotorRamp(1, 1, 255);   // Motor 1 (levý): forward, 100%
  startMotorRamp(2, -1, 255);  // Motor 2 (pravý): backward, 100%
}

void processCommand(const String& cmd) {
  Serial.println(cmd);

  if (cmd == "stop") {
    emergencyStop();
  }
  else if (cmd == "rope_out") { /* výstup */ }
  else if (cmd == "rope_in")  { /* výstup */ }
  else if (cmd == "comb_out") { /* výstup */ }
  else if (cmd == "comb_in")  { /* výstup */ }
  else if (cmd == "turn_left")  { /* výstup */ }
  else if (cmd == "turn_right") { /* výstup */ }
  else if (cmd == "forward") { moveForward(); }
  else if (cmd == "back")    { moveBackward(); }
  else if (cmd == "left")    { moveLeft(); }
  else if (cmd == "right")   { moveRight(); }
  else if (cmd == "spoon_open")  { /* výstup */ }
  else if (cmd == "spoon_close") { /* výstup */ }
  else if (cmd == "L1") { /* světlo */ }
  else if (cmd == "L2") { /* světlo */ }
  else if (cmd == "L3") { /* světlo */ }
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

void loop() {
  handleClient();
  watchdogCheck();
  updateMotorRamp();  // Continuously update motor PWM ramps
}
