#include <Wire.h>
#include <BH1750.h>
#include <VL53L0X.h>

BH1750 lightMeter;
VL53L0X distanceSensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // ---- BH1750 init ----
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 init FAILED (check wiring/address)");
    while (1) { delay(10); }
  }

  // ---- VL53L0X init ----
  distanceSensor.setTimeout(500);
  if (!distanceSensor.init()) {
    Serial.println("VL53L0X init FAILED (check wiring/address)");
    while (1) { delay(10); }
  }

  // Optional: tune for stability
  distanceSensor.setMeasurementTimingBudget(20000); // 20 ms budget
  distanceSensor.startContinuous(50);               // new reading about every 50 ms

  Serial.println("Both sensors initialized OK");
}

void loop() {
  // Read BH1750
  float lux = lightMeter.readLightLevel();

  // Read VL53L0X
  uint16_t mm = distanceSensor.readRangeContinuousMillimeters();

  // Print in a single clean line (easy for Python/PHP to parse later)
  Serial.print("L:");
  Serial.print(lux, 1);
  Serial.print(" lx; ");

  Serial.print("D:");
  Serial.print(mm);
  Serial.print(" mm");

  if (distanceSensor.timeoutOccurred()) {
    Serial.print(" ; VL53:TIMEOUT");
  }

  Serial.println();

  delay(200); // ~5 updates/sec
}
