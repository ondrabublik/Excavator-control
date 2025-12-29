#include <WiFiS3.h>
#include "webpage.h"
#include "led.h"
#include "Arduino_LED_Matrix.h"

const char* ssid = "excavator";
const char* password = "password";

WiFiServer server(80);
ArduinoLEDMatrix matrix;

// ===== H-BRIDGE 1: L298N - TRACK MOTORS =====
// Motor 1 (levý pás)
#define IN1 2  // forward
#define IN2 4  // backward
#define ENA 3  // PWM

// Motor 2 (pravý pás)
#define IN3 5  // forward
#define IN4 6  // backward
#define ENB 3  // PWM

// ===== H-BRIDGE 2: L298N - ROPE & COMB MOTORS =====
// Motor 3 (lano - rope)
#define IN5 7  // Motor direction pin 5
#define IN6 8 // Motor direction pin 6
#define ENC 3 // PWM

// Motor 4 (hřeben - comb)
#define IN7 9 // Motor direction pin 7
#define IN8 10 // Motor direction pin 8
#define END 3 // PWM

// ===== H-BRIDGE 3: L298N - ROTATION MOTOR =====
// Motor 5 (otáčení - turn)
#define IN9 11  // Motor direction pin 9
#define IN10 12 // Motor direction pin 10
#define ENE 3 // PWM

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
MotorState motor3 = {0, 0, 0, false};  // rope
MotorState motor4 = {0, 0, 0, false};  // comb
MotorState motor5 = {0, 0, 0, false};  // turn

// ===== WATCHDOG =====
unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 400; // ms

void emergencyStop() {
  // STOP ALL MOTORS IMMEDIATELY
  // H-Bridge 1 (tracks)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, 0);
  
  // H-Bridge 2 (rope & comb)
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(ENC, 0);
  
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  digitalWrite(END, 0);
  
  // H-Bridge 3 (turn)
  digitalWrite(IN9, LOW);
  digitalWrite(IN10, LOW);
  digitalWrite(ENE, 0);
  
  motor1.isActive = false;
  motor2.isActive = false;
  motor3.isActive = false;
  motor4.isActive = false;
  motor5.isActive = false;
  motor1.currentSpeed = 0;
  motor2.currentSpeed = 0;
  motor3.currentSpeed = 0;
  motor4.currentSpeed = 0;
  motor5.currentSpeed = 0;
  
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
      motor2.currentSpeed = (motor2.targetSpeed * elapsed) / PWM_RAMP_TIME;
    }
    analogWrite(ENB, motor2.currentSpeed);
  }

  // Update motor 3 (rope)
  if (motor3.isActive) {
    unsigned long elapsed = now - motor3.startTime;
    if (elapsed >= PWM_RAMP_TIME) {
      motor3.currentSpeed = motor3.targetSpeed;
    } else {
      motor3.currentSpeed = (motor3.targetSpeed * elapsed) / PWM_RAMP_TIME;
    }
    analogWrite(ENC, motor3.currentSpeed);
  }

  // Update motor 4 (comb)
  if (motor4.isActive) {
    unsigned long elapsed = now - motor4.startTime;
    if (elapsed >= PWM_RAMP_TIME) {
      motor4.currentSpeed = motor4.targetSpeed;
    } else {
      motor4.currentSpeed = (motor4.targetSpeed * elapsed) / PWM_RAMP_TIME;
    }
    analogWrite(END, motor4.currentSpeed);
  }

  // Update motor 5 (turn)
  if (motor5.isActive) {
    unsigned long elapsed = now - motor5.startTime;
    if (elapsed >= PWM_RAMP_TIME) {
      motor5.currentSpeed = motor5.targetSpeed;
    } else {
      motor5.currentSpeed = (motor5.targetSpeed * elapsed) / PWM_RAMP_TIME;
    }
    analogWrite(ENE, motor5.currentSpeed);
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
  } else if (motorNum == 3) {  // rope motor
    if (direction == 1) {
      digitalWrite(IN5, HIGH);
      digitalWrite(IN6, LOW);
    } else if (direction == -1) {
      digitalWrite(IN5, LOW);
      digitalWrite(IN6, HIGH);
    } else {
      digitalWrite(IN5, LOW);
      digitalWrite(IN6, LOW);
    }
  } else if (motorNum == 4) {  // comb motor
    if (direction == 1) {
      digitalWrite(IN7, HIGH);
      digitalWrite(IN8, LOW);
    } else if (direction == -1) {
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, HIGH);
    } else {
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);
    }
  } else if (motorNum == 5) {  // turn motor
    if (direction == 1) {
      digitalWrite(IN9, HIGH);
      digitalWrite(IN10, LOW);
    } else if (direction == -1) {
      digitalWrite(IN9, LOW);
      digitalWrite(IN10, HIGH);
    } else {
      digitalWrite(IN9, LOW);
      digitalWrite(IN10, LOW);
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
  } else if (motorNum == 3) {  // rope
    motor3.targetSpeed = speed;
    motor3.startTime = millis();
    motor3.isActive = true;
    motor3.currentSpeed = 0;
  } else if (motorNum == 4) {  // comb
    motor4.targetSpeed = speed;
    motor4.startTime = millis();
    motor4.isActive = true;
    motor4.currentSpeed = 0;
  } else if (motorNum == 5) {  // turn
    motor5.targetSpeed = speed;
    motor5.startTime = millis();
    motor5.isActive = true;
    motor5.currentSpeed = 0;
  }
}

void moveForward() {
  startMotorRamp(1, 1, 255);  // Motor 1: forward, 100%
  startMotorRamp(2, 1, 255);  // Motor 2: forward, 100%
  matrix.clear();
  matrix.renderBitmap(FORWARD, MAX_Y, MAX_X);
}

void moveBackward() {
  startMotorRamp(1, -1, 255);  // Motor 1: backward, 100%
  startMotorRamp(2, -1, 255);  // Motor 2: backward, 100%
  matrix.clear();
  matrix.renderBitmap(BACKWARD, MAX_Y, MAX_X);
}

void moveLeft() {
  startMotorRamp(1, -1, 255);  // Motor 1 (levý): backward, 100%
  startMotorRamp(2, 1, 255);   // Motor 2 (pravý): forward, 100%
  matrix.clear();
  matrix.renderBitmap(LEFT, MAX_Y, MAX_X);
}

void moveRight() {
  startMotorRamp(1, 1, 255);   // Motor 1 (levý): forward, 100%
  startMotorRamp(2, -1, 255);  // Motor 2 (pravý): backward, 100%
  matrix.clear();
  matrix.renderBitmap(RIGHT, MAX_Y, MAX_X);
}

void moveRopeOut() {
  startMotorRamp(3, 1, 255);  // Motor 3: extend rope, 100%
  matrix.clear();
  matrix.renderBitmap(ROPE_OUT, MAX_Y, MAX_X);
}

void moveRopeIn() {
  startMotorRamp(3, -1, 255);  // Motor 3: retract rope, 100%
  matrix.clear();
  matrix.renderBitmap(ROPE_IN, MAX_Y, MAX_X);
}

void moveCombOut() {
  startMotorRamp(4, 1, 255);  // Motor 4: extend comb, 100%
  matrix.clear();
  matrix.renderBitmap(COMB_OUT, MAX_Y, MAX_X);
}

void moveCombIn() {
  startMotorRamp(4, -1, 255);  // Motor 4: retract comb, 100%
  matrix.clear();
  matrix.renderBitmap(COMB_IN, MAX_Y, MAX_X);
}

void turnLeft() {
  startMotorRamp(5, -1, 255);  // Motor 5: turn left, 100%
  matrix.clear();
  matrix.renderBitmap(TURN_LEFT, MAX_Y, MAX_X);
}

void turnRight() {
  startMotorRamp(5, 1, 255);   // Motor 5: turn right, 100%
  matrix.clear();
  matrix.renderBitmap(TURN_RIGHT, MAX_Y, MAX_X);
}

void spoon_function(int open) {
  if (open == 1) {
    matrix.clear();
    matrix.renderBitmap(SO, MAX_Y, MAX_X);
  } else {
    matrix.clear();
    matrix.renderBitmap(SC, MAX_Y, MAX_X);
  }
}

void L1_function() {
  matrix.clear();
  matrix.renderBitmap(L1, MAX_Y, MAX_X);
}

void L2_function() {
  matrix.clear();
  matrix.renderBitmap(L2, MAX_Y, MAX_X);
}

void L3_function() {
  matrix.clear();
  matrix.renderBitmap(L3, MAX_Y, MAX_X);
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

  // Initialize H-Bridge 1 (track motors)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize H-Bridge 2 (rope & comb motors)
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(END, OUTPUT);

  // Initialize H-Bridge 3 (turn motor)
  pinMode(IN9, OUTPUT);
  pinMode(IN10, OUTPUT);
  pinMode(ENE, OUTPUT);

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
}
