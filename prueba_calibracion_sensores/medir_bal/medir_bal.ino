#include <Arduino.h>
#include "HX711.h"

#define DT_PIN  4
#define SCK_PIN 5

HX711 scale;
float calibration_factor = 428.473;

void setup() {
  Serial.begin(115200);
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);

  Serial.println("Taring...");
  scale.tare();
  Serial.println("Tare complete.");
}

void loop() {
  float reading1 = scale.get_units(5);   // fewer samples = faster
  delay(100);                             // shorter wait between checks
  float reading2 = scale.get_units(5);

  if (abs(reading1 - reading2) < 1.0) {
    Serial.print("Weight: ");
    Serial.print(reading2, 2);
    Serial.println(" g");
  }

  delay(300); // shorter pause before next cycle
}