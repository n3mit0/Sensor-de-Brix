#include "HX711.h"

// Pines de conexiÃ³n al HX711
const int DT_PIN  = 4;
const int SCK_PIN = 5;

HX711 scale;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Iniciando HX711...");
  scale.begin(DT_PIN, SCK_PIN);

  Serial.println("Quita cualquier peso de la celda.");
  delay(3000);

  // Tara: promedia varias lecturas y fija el punto cero (offset)
  scale.tare(20);
  long offset = scale.get_offset();
  Serial.print("Offset (raw sin peso): ");
  Serial.println(offset);

  Serial.println();
  Serial.println("Ahora coloca un peso CONOCIDO sobre la celda.");
  Serial.println("Ese peso lo necesitarÃ¡s para el cÃ¡lculo del factor.");
  Serial.println("Las lecturas RAW comenzarÃ¡n en 5 segundos...");
  delay(5000);
}

void loop() {
  if (scale.is_ready()) {
    // Promedio de 10 lecturas raw (sin escalar, sin restar offset)
    long raw = scale.read_average(10);

    // Valor raw ya con el offset restado (esto es lo que usarÃ¡s en la fÃ³rmula)
    long raw_menos_offset = raw - scale.get_offset();

    Serial.print("RAW: ");
    Serial.print(raw);
    Serial.print("   |   RAW - offset: ");
    Serial.println(raw_menos_offset);
  } else {
    Serial.println("HX711 no listo (revisa conexiones)");
  }

  delay(500);
}