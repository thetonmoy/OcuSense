# VL53L0X Only

This sketch reads **distance** from the VL53L0X Time-of-Flight sensor over I²C and prints the value to Serial Monitor.

## What you need
- Arduino Nano (ATmega328P)
- VL53L0X breakout module
- Jumper wires

## Wiring (Nano ↔ VL53L0X)
Typical I²C wiring:
- **SDA → A4**
- **SCL → A5**
- **GND → GND**
- **VCC / VIN / 5V → 5V** (use whichever power pin your module provides)

> If your module is labeled **3V3 only**, connect to **Nano 3.3V**. Otherwise most breakout boards accept **5V** input.

## Arduino IDE settings (important)
- **Tools → Board:** Arduino Nano  
- **Tools → Processor:** **ATmega328P**
- **Tools → Port:** your Nano COM port (e.g., COM4)
- **Serial Monitor baud:** `9600` (must match `Serial.begin(...)` in the sketch)

## Library install
Install **VL53L0X** library by Pololu:
- Arduino IDE → **Tools → Manage Libraries**
- Search: **VL53L0X**
- Install: **VL53L0X by Pololu**

## Upload & test
1. Open `vl53l0x_only.ino`
2. Upload to Nano
3. Open **Serial Monitor** at **9600 baud**
4. You should see distance values (mm), e.g.:
   - `Distance: 612 mm`

## References
- RoboticsBD VL53L0X product page: https://store.roboticsbd.com/sensors/554-gy-vl53l0xv2v-vl53l0x-time-of-flight-distance-sensor-breakout-module-robotics-bangladesh.html
