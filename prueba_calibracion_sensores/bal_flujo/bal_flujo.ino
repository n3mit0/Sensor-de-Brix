#include <Arduino.h>
#include "HX711.h"

#define DT_PIN   4
#define SCK_PIN  5
#define FLOW_PIN 15   // GPIO D2 - YF-S401

HX711 scale;
float calibration_factor = 428.473;

// --- Variables sensor de flujo ---
volatile unsigned long pulseCount = 0;
float PULSES_PER_LITER = 1777.0;   // Ajustar según datasheet/calibración de tu unidad
unsigned long lastFlowCalc = 0;
float flowRate_Lmin = 0.0;
float totalVolume_L = 0.0;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  Celda de carga
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);
  Serial.println("Taring...");
  scale.tare();
  Serial.println("Tare complete.");

  // Sensor de flujo
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);

  lastFlowCalc = millis();
}

void loop() {
  /// --- Lectura de la celda de carga ---
  float reading1 = scale.get_units(5);
  delay(100);
  float reading2 = scale.get_units(5);

  if (abs(reading1 - reading2) < 1.0) {
    Serial.print("Weight: ");
    Serial.print(reading2, 2);
    Serial.println(" g");
  }

  // --- Cálculo del flujo cada 1 segundo ---
  unsigned long now = millis();
  if (now - lastFlowCalc >= 1000) {
    // Deshabilitar interrupción brevemente para leer el contador de forma segura
    detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);

    float elapsedSec = (now - lastFlowCalc) / 1000.0;
    lastFlowCalc = now;

    // Litros en este intervalo
    float litersThisInterval = pulses / PULSES_PER_LITER;
    totalVolume_L += litersThisInterval;

    // Caudal en L/min
    flowRate_Lmin = (litersThisInterval / elapsedSec) * 60.0;

    Serial.print("Flow: ");
    Serial.print(flowRate_Lmin, 3);
    Serial.print(" L/min | Total: ");
    Serial.print(totalVolume_L, 3);
    Serial.println(" L");
  }

  delay(300);
}