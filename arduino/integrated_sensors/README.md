# Integrated Sensors (BH1750 + VL53L0X)

This sketch reads:
- **BH1750** → ambient light (**lux**)
- **VL53L0X** → distance (**mm**)

Both sensors share the same **I²C bus** (A4/A5), and values are printed in one serial line (easy to parse later).

## What you need
- Arduino Nano (ATmega328P)
- BH1750 (GY-30)
- VL53L0X breakout module
- Jumper wires

## Wiring (Nano ↔ Sensors)
### Shared I²C lines
- **A4 ↔ BH1750 SDA ↔ VL53L0X SDA**
- **A5 ↔ BH1750 SCL ↔ VL53L0X SCL**

### Power
- **5V ↔ BH1750 VCC**
- **GND ↔ both GND**
- VL53L0X power pin depends on your board label:
  - **VIN / 5V / VCC → Nano 5V**
  - If labeled **3V3 only**, connect to **Nano 3.3V**

### BH1750 address pin
- **ADD**: leave unconnected (commonly uses I²C address `0x23`)

## Arduino IDE settings (important)
- **Tools → Board:** Arduino Nano  
- **Tools → Processor:** **ATmega328P**
- **Tools → Port:** your Nano COM port (e.g., COM4)
- **Serial Monitor baud:** `9600` (must match `Serial.begin(...)` in the sketch)

## Libraries
Install both from Arduino Library Manager:
- **BH1750 by Christopher Laws**
- **VL53L0X by Pololu**

## Output format (example)
The sketch prints:
- `L:178.0 lx; D:612 mm`

## References
- RoboticsBD BH1750: https://store.roboticsbd.com/sensors/429-bh1750fvi-digital-light-intensity-sensor-robotics-bangladesh.html
- RoboticsBD VL53L0X: https://store.roboticsbd.com/sensors/554-gy-vl53l0xv2v-vl53l0x-time-of-flight-distance-sensor-breakout-module-robotics-bangladesh.html
