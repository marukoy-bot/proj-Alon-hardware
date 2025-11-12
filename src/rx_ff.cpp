#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <LiquidCrystal_I2C.h>

#define NSS 5
#define RST 32
#define DIO0 33

String g_pos = "---";
String g_dist = "---";

LiquidCrystal_I2C lcd(0x27, 20, 4);

// LoRa link status
unsigned long lastLoRaTime = 0;
bool loraConnected = false;

void setup() {
    // Initialize USB Serial at 115200 baud FIRST, before LCD
    Serial.begin(115200);
    delay(100); // Give Serial time to initialize
    
    // LoRa setup
    LoRa.setPins(NSS, RST, DIO0);
    pinMode(2, OUTPUT);
    
    if (!LoRa.begin(433E6)) {
        Serial.println("{\"error\":\"LoRa init failed\"}");
        while(1);
    }
    
    // LCD setup - AFTER Serial
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0); lcd.print("LoRa: disconnected");
    lcd.setCursor(0,1); lcd.print("Pos: ---");
    lcd.setCursor(0,2); lcd.print("Dist: ---");
    
    Serial.println("{\"status\":\"ready\"}");
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

void sendJsonData(String angle, String distance) {
    // Send ONLY JSON data to Serial (USB)
    Serial.print("{\"angle\":");
    Serial.print(angle);
    Serial.print(",\"distance\":");
    Serial.print(distance);
    Serial.print(",\"connected\":");
    Serial.print(loraConnected ? "true" : "false");
    Serial.println("}");
    Serial.flush(); // Make sure data is sent immediately
}

void loop() {
    // Check for incoming commands from Android app
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "STATUS") {
            Serial.print("{\"status\":\"running\",\"angle\":\"");
            Serial.print(g_pos);
            Serial.print("\",\"distance\":\"");
            Serial.print(g_dist);
            Serial.print("\",\"connected\":");
            Serial.print(loraConnected ? "true" : "false");
            Serial.println("}");
            Serial.flush();
        }
        else if (command == "PING") {
            Serial.println("{\"pong\":true}");
            Serial.flush();
        }
    }

    // Read LoRa messages
    String message = readLoRaMessage();
    if (message.length() > 0) {
        message.trim();

        int start = 0;
        int commaIndex = -1;
        String lora_id = "";
        String pos = "";
        String dist = "";

        // Parse: lora_id, position, distance
        commaIndex = message.indexOf(',', start);
        if (commaIndex == -1) {
            lora_id = message.substring(start);
        } else {
            lora_id = message.substring(start, commaIndex);
            start = commaIndex + 1;
        }
        lora_id.trim();

        if (lora_id == "0x01") {
            // Parse position
            commaIndex = message.indexOf(',', start);
            if (commaIndex == -1) {
                pos = message.substring(start);
            } else {
                pos = message.substring(start, commaIndex);
                start = commaIndex + 1;
            }
            pos.trim();

            // Parse distance
            commaIndex = message.indexOf(',', start);
            if (commaIndex == -1) {
                dist = message.substring(start);
            } else {
                dist = message.substring(start, commaIndex);
            }
            dist.trim();

            // Update only if values changed
            if (g_pos != pos || g_dist != dist) {
                g_pos = pos;
                g_dist = dist;

                // Update LoRa connection status
                lastLoRaTime = millis();
                if (!loraConnected) {
                    loraConnected = true;
                    lcd.setCursor(0,0);
                    lcd.print("LoRa: connected   ");
                }

                // Update LCD
                lcd.setCursor(0,1);
                lcd.print("Pos: " + g_pos + (char)223 + "   ");
                lcd.setCursor(0,2);
                lcd.print("Dist: " + g_dist + " cm   ");

                // Send data to Android app via USB Serial
                sendJsonData(pos, dist);
            }
        }
    }

    // Check for LoRa disconnection (5s timeout)
    if (loraConnected && (millis() - lastLoRaTime > 5000)) {
        loraConnected = false;
        lcd.setCursor(0,0);
        lcd.print("LoRa: disconnected");
        
        // Notify app of disconnection
        Serial.println("{\"connected\":false}");
        Serial.flush();
    }

    delay(10);
}