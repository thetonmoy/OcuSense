#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("VL53L0X test starting...");

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    Serial.println("Check wiring (SDA=A4, SCL=A5, GND, VCC/VIN) and I2C address.");
    while (1) { delay(10); }
  }

  // Optional: improve stability / range on many modules
  sensor.setMeasurementTimingBudget(20000); // 20ms (try 33000 or 50000 if noisy)

  // Continuous mode (updates repeatedly)
  sensor.startContinuous(50); // one measurement every ~50ms

  Serial.println("VL53L0X initialized OK");
}

void loop() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("TIMEOUT");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

  delay(200);
}
