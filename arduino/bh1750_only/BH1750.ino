#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Start BH1750 in Continuous High Resolution mode
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 initialized OK");
  } else {
    Serial.println("BH1750 init FAILED (check wiring / I2C)");
    while (1) { delay(10); } // stop here
  }
}

void loop() {
  float lux = lightMeter.readLightLevel();

  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");

  delay(500); // update twice per second
}
