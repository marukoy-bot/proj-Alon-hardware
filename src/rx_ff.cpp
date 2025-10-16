#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#define DEBUG 1
#include "debug.h"

#define NSS 5
#define RST 32
#define DIO0 33
#define WS_PORT 81

const char* ap_ssid = "ESP32 RX";
const char* ap_password = "00000000";

String g_pos = "___";      // for LCD
String g_pos_web = "___";  // for web (UTF-8)
String g_dist = "___";

LiquidCrystal_I2C lcd(0x27, 20, 4);

WebServer server(80);
WebSocketsServer webSocket(WS_PORT);

// ---------------- LoRa link status ----------------
unsigned long lastLoRaTime = 0;
bool loraConnected = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>ESP32 LoRa RX</title>
  <style>
    body { font-family: Arial, sans-serif; text-align:center; margin-top:40px; }
    .card { display:inline-block; padding:20px 30px; border-radius:8px; box-shadow: 0 2px 8px rgba(0,0,0,0.2); }
    h1 { margin:0 0 10px 0; }
    .value { font-size:2.4rem; margin:10px 0; }
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 LoRa Receiver</h1>
    <div>Position (angle): <div id="pos" class="value">--</div></div>
    <div>Distance: <div id="dist" class="value">--</div></div>
    <p><small>Connect to Wi-Fi SSID: <strong>ESP32 RX</strong></small></p>
  </div>

<script>
  var gateway = "192.168.4.1";
  var ws;
  function initWS() {
    ws = new WebSocket("ws://" + gateway + ":81/");
    ws.onopen = function() { console.log("WS open"); };
    ws.onclose = function() { console.log("WS closed, retry in 2s"); setTimeout(initWS, 2000); };
    ws.onmessage = function(evt) {
      try {
        var obj = JSON.parse(evt.data);
        if (obj.pos !== undefined) document.getElementById('pos').innerHTML = obj.pos;
        if (obj.dist !== undefined) document.getElementById('dist').innerText = obj.dist;
      } catch(e) { console.log("parse err", e); }
    };
  }
  window.onload = initWS;
</script>
</body>
</html>
)rawliteral";

void broadcastUpdate() {
  String payload = "{\"pos\":\"" + g_pos_web + "\",\"dist\":\"" + g_dist + "\"}";
  webSocket.broadcastTXT(payload);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    broadcastUpdate();
  }
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void setup() {
#if DEBUG == 1
    Serial.begin(115200);
#endif
    LoRa.setPins(NSS, RST, DIO0);
    pinMode(2, OUTPUT);
    
    if (!LoRa.begin(433E6)) {
        while(1);
    }
    Serial.println("LoRa ready.");

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0,0); lcd.print("LoRa: disconnected");
    lcd.setCursor(0,1); lcd.print("Pos: ---");
    lcd.setCursor(0,2); lcd.print("Dist: ---");

    WiFi.softAP(ap_ssid, ap_password);
    IPAddress ip = WiFi.softAPIP();
    db_print("AP IP address: ");
    db_println(ip);

    server.on("/", handleRoot);
    server.begin();
    db_println("HTTP server started");

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    db_println("WebSocket server started on port " + String(WS_PORT));
}

String readLoRaMessage() {
  String message = "";
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }
  }
  return message;
}

void loop() {
    server.handleClient();
    webSocket.loop();

    String message = readLoRaMessage();
    if (message.length() > 0) {
        message.trim();
        db_print("data received: ");
        db_println(message);

        int start = 0;
        int commaIndex = -1;
        String lora_id = "";
        String pos = "";
        String dist = "";

        // first token -> lora_id
        commaIndex = message.indexOf(',', start);
        if (commaIndex == -1) {
          lora_id = message.substring(start);
        } else {
          lora_id = message.substring(start, commaIndex);
          start = commaIndex + 1;
        }
        lora_id.trim();

        if (lora_id == "0x01") {
          // second token -> pos
          commaIndex = message.indexOf(',', start);
          if (commaIndex == -1) {
              pos = message.substring(start);
          } else {
              pos = message.substring(start, commaIndex);
              start = commaIndex + 1;
          }
          pos.trim();

          // third token -> dist
          commaIndex = message.indexOf(',', start);
          if (commaIndex == -1) {
              dist = message.substring(start);
          } else {
              dist = message.substring(start, commaIndex);
          }
          dist.trim();
          if (dist.length() > 0) dist += " cm";

          // update only if values changed
          if (g_pos != pos || g_dist != dist) {
              g_pos = pos + (char)223;   // for LCD
              g_pos_web = pos + "\u00B0"; // for web
              g_dist = dist;

              db_println("Parsed -> pos: " + g_pos + " dist: " + g_dist);

              // --- LoRa connected handling ---
              lastLoRaTime = millis();
              if (!loraConnected) {
                  loraConnected = true;
                  lcd.setCursor(0,0);
                  lcd.print("LoRa: connected   ");
              }

              // update LCD pos/dist
              lcd.setCursor(0,1);
              lcd.print("Pos: " + g_pos + "   ");
              lcd.setCursor(0,2);
              lcd.print("Dist: " + g_dist + "   ");

              // push update to website
              broadcastUpdate();
          }
        }
    }

    // --- check for LoRa disconnection (5s timeout) ---
    if (loraConnected && (millis() - lastLoRaTime > 5000)) {
        loraConnected = false;
        lcd.setCursor(0,0);
        lcd.print("LoRa: disconnected");
    }

    // --- periodic broadcast (safety net) ---
    static unsigned long lastBroadcast = 0;
    if (millis() - lastBroadcast > 1000) {   // send every 1s
        broadcastUpdate();
        lastBroadcast = millis();
    }

    delay(10);
}
