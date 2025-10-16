#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LoRa.h>
#include <SPI.h>
#include <AccelStepper.h>

#define DEBUG 1
#include "debug.h"

#define lora_id "0x01"

#define in1 27
#define in2 14
#define in3 12
#define in4 13

#define rx 21    // tx (white)
#define tx 22   // rx (yellow)

#define nss 5
#define mosi 23
#define miso 19
#define sck 18
#define rst 32
#define dio0 33
#define motor_interface_type 4

const int stepsPerRevolution = 2048;
int stepper_pos = 0;
unsigned char data[4] = {0};
long duration;

AccelStepper stepper(motor_interface_type, in1, in3, in2, in4);
SoftwareSerial us(rx, tx);

TaskHandle_t Task1;
TaskHandle_t Task2;

void offStepper();
float getDistance();
void handleLoRa(void * pvParameters);
void handleStepper(void * pvParameters);

void setup() {
    stepper.setMaxSpeed(1000.0);
    stepper.setAcceleration(50.0);
    stepper.setSpeed(200);
    stepper.moveTo(2048);
    
#if DEBUG == 1
    Serial.begin(115200);
#endif
    us.begin(9600);

    SPI.begin(sck, miso, mosi, nss);
    LoRa.setPins(nss, rst, dio0);
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa initialization failed.");
        while(1);
    }
    Serial.println("LoRa tx ready.");

    xTaskCreatePinnedToCore(
        handleLoRa,
        "handleLoRa",
        10000,
        NULL,
        1,
        &Task1,
        0
    );
    vTaskDelay(500);

        xTaskCreatePinnedToCore(
        handleStepper,
        "handleStepper",
        10000,
        NULL,
        1,
        &Task2,
        1
    );
    vTaskDelay(500);

}

void loop() {
    
}

void offStepper() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

float getDistance() {
    float dist = 0;
    us.flush();
    delay(30);
    digitalWrite(tx, HIGH);
    delay(30);
    digitalWrite(tx, LOW);
    delay(30);
    digitalWrite(tx, HIGH);
    delay(60);

    for (int i = 0; i < 4; i++) {
        data[i] = us.read();
    }

    // Raw distance from sensor (calculated for air)
    dist = (data[1] * 256) + data[2];
    dist = dist / 10.0;   // cm in air

    // Convert to equivalent distance in water
    dist = dist * (1482.0 / 343.0);   // ≈ dist * 4.32

    return dist;   // distance in cm (water medium)
}

void sendLoRa(float distance) {
    LoRa.beginPacket();
    LoRa.print(String(lora_id) + "," + String(distance));
    LoRa.endPacket();
}

void handleLoRa(void * pvParameters) {
    Serial.println("Task1 running on Core " + String(xPortGetCoreID()));

    for (;;) {
        // Convert stepper steps to degrees
        float angle = (stepper_pos / 2048.0f) * 360.0f;

        // Normalize angle to 0–360
        angle = fmod(angle, 360.0f);
        if (angle < 0) angle += 360.0f;

        float distance = getDistance();

        // Send LoRa packet
        LoRa.beginPacket();
        LoRa.print(String(lora_id) + "," + String((int)angle) + "," + String(distance));
        LoRa.endPacket();

        // Debug
        Serial.println(String(lora_id) + "," + String((int)angle) + "," + String(distance));

        vTaskDelay(250 / portTICK_PERIOD_MS);   // 4 Hz transmissions
    }
}

void handleStepper(void * pvParameters) {
    Serial.println("Task2 running on Core " + xPortGetCoreID());

    for (;;) {       
        if (stepper.distanceToGo() == 0) {
            if (stepper.currentPosition() == 0) {
                stepper.moveTo(2048);   // go forward 1 revolution
            } else {
                stepper.moveTo(0);      // return to zero
            }
            stepper_pos = stepper.currentPosition();
        }


        stepper.run();
        stepper_pos = stepper.currentPosition();

        vTaskDelay(1);
    }
}