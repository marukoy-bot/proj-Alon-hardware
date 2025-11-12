# Alon
A fish finding device using ultrasonic sensor to detect fishes and LoRa to transmit data over long distances.

# Schematics
Receiver (RX)

<div align="center">
    <image src="alon_rx_bb.png" alt="Alon RX"/>
</div>

Transmitter (TX)

<div align="center">
    <image src="alon_tx_bb.png" alt="Alon RX"/>
</div>

# App Interfacing
It connects to the [Alon Android app](https://github.com/marukoy-bot/Alon) via USB OTG. 

# Components
| TX | RX |
| --- | --- |
| ESP32 <br> JSN-SR04T Ultrasonic Sensor <br> SX1278 LoRa <br> Solar System <br> 28BYJ-48 5v Stepper Motor <br> ULN2003 driver | ESP32 <br> SX1278 LoRa <br> 20x4 IIC LCD <br> |