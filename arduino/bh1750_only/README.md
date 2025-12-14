---
title: BH1750 (GY-30) — Standalone Test (Arduino Nano)

---

# BH1750 Only

This sketch reads **ambient light (lux)** from the BH1750 (GY-30) using I²C and prints the value to Serial Monitor.

## What you need
- Arduino Nano (ATmega328P)
- BH1750 module
- Jumper wires

## Wiring (Nano ↔ BH1750)
- **VCC → 5V**
- **GND → GND**
- **SCL → A5**
- **SDA → A4**
- **ADD**: leave unconnected (commonly uses I²C address `0x23`)

## Arduino IDE settings (important)
- **Tools → Board:** Arduino Nano  
- **Tools → Processor:** **ATmega328P**
- **Tools → Port:** your Nano COM port (e.g., COM4)
- **Serial Monitor baud:** `9600` (must match `Serial.begin(...)` in the sketch)

## Library install
Install **BH1750** library by *Christopher Laws (claws)*:
- Arduino IDE → **Tools → Manage Libraries**
- Search: **BH1750**
- Install: **BH1750 by Christopher Laws**

## Upload & test
1. Open `bh1750_only.ino`
2. Upload to Nano
3. Open **Serial Monitor** at **9600 baud**
4. You should see something like:
   - `Light: 178 lx`

## References
- RoboticsBD BH1750 product page: https://store.roboticsbd.com/sensors/429-bh1750fvi-digital-light-intensity-sensor-robotics-bangladesh.html
