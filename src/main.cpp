#include <WiFiS3.h>
#include "webpage.h"

const char* ssid = "excavator";
const char* password = "password";

WiFiServer server(80);

// ===== WATCHDOG =====
unsigned long lastCmdTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 300; // ms

void setup() {
  Serial.begin(115200);

  WiFi.beginAP(ssid, password);
  delay(2000);

  Serial.print("AP IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void emergencyStop() {
  // ZDE VYPNI VŠECHNY MOTORY
  Serial.println("!!! WATCHDOG STOP !!!");
}

void watchdogCheck() {
  if (millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
    emergencyStop();
    lastCmdTime = millis();
  }
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
  else if (cmd == "forward") { /* výstup */ }
  else if (cmd == "back")    { /* výstup */ }
  else if (cmd == "left")    { /* výstup */ }
  else if (cmd == "right")   { /* výstup */ }
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
}
