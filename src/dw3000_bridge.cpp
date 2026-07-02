// Interactive UART bridge for manually exercising the Makerfabs DW3000 AOA anchor's command console.
// Standalone entry point (env:dw3000-bridge) — does not touch the car's main.cpp or existing UWB driver.
// Relays raw bytes both directions between Serial (USB, to `pio device monitor`) and dwSerial (UART1,
// to the anchor's TXD1/RXD1 header pins) with zero interpretation, so commands can be typed and
// responses read directly without rebuilding firmware for each attempt.
#include <Arduino.h>

// Anchor UART wiring: TXD1 (anchor) -> RX_PIN, RXD1 (anchor) -> TX_PIN, shared GND. Anchor powered separately via its own USB-C.
static const int RX_PIN = 4;
static const int TX_PIN = 5;

static HardwareSerial dwSerial(1);

void setup() {
    delay(3000);
    Serial.begin(115200);
    dwSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("=== DW3000 UART bridge ready — type a command and press Enter to send ===");
}

// Forwards every byte available in either direction — no framing, no parsing.
void loop() {
    while (Serial.available())   dwSerial.write(Serial.read());
    while (dwSerial.available()) Serial.write(dwSerial.read());
}
