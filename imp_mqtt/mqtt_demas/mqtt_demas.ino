#include <WiFi.h>
#include <PubSubClient.h>

#include <Arduino.h>
#include "HX711.h"

// =========================
// ENTRADAS SENSORES
// =========================
#define DT_PIN 4
#define SCK_PIN 5
#define FLOW_PIN 15  // GPIO D15 - YF-S401

// =========================
// DATOS DE WIFI
// =========================
const char* ssid = "realme C67";
const char* password = "jkgh23!;";

// =========================
// DATOS DEL BROKER MQTT
// =========================
const char* mqtt_server = "10.250.214.191";  // IP del PC donde corre Mosquitto
const int mqtt_port = 1883;
const char* mqtt_topic = "kalman_proj";  //nombre


// =========================
// SENSOR BALANZA
// =========================
bool loadCellConnected = false;  //para revisar conexion de celda
HX711 scale;
float calibration_factor = 428.473;

// --- Variables sensor de flujo ---
volatile unsigned long pulseCount = 0;
float PULSES_PER_LITER = 1777.0;  // Ajustar según datasheet/calibración de tu unidad
unsigned long lastFlowCalc = 0;
float flowRate_Lmin = 0.0;
float totalVolume_L = 0.0;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// =========================
// VARIABLES DE MUESTREO
// =========================
int totalMuestras = 5;
int contador = 0;

// =========================
// OBJETOS WIFI Y MQTT
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

// =========================
// FUNCION: CONECTAR WIFI
// =========================
void setup_wifi() {
  delay(100);
  Serial.println();
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi conectado");
  Serial.print("IP de la ESP32: ");
  Serial.println(WiFi.localIP());
}

// =========================
// FUNCION: RECONECTAR MQTT
// =========================
void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando al broker MQTT... ");

    String clientId = "ESP32Client-";
    clientId += String(random(1000, 9999));

    if (client.connect(clientId.c_str())) {
      Serial.println("conectado");
    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentando nuevamente en 2 segundos");
      delay(2000);
    }
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(2000);

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Inicio de adquisicion por MQTT");


  // Celda de carga
  scale.begin(DT_PIN, SCK_PIN);
  Serial.println("Checking load cell...");
  if (scale.wait_ready_timeout(200)) {  // espera max 200 ms, no infinito
    loadCellConnected = true;
    scale.set_scale(calibration_factor);
    Serial.println("Taring...");
    scale.tare();
    Serial.println("Tare complete.");
  } else {
    Serial.println("Load cell NOT detected. Reinicia la ESP32 despues de conectarla.");
  }


  // Sensor de flujo
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);

  lastFlowCalc = millis();
}

// =========================
// LOOP
// =========================
void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  if (contador < totalMuestras) {

    /// --- Lectura de la celda de carga ---
    float reading2 = 0;
    static float lastValidWeight = 0;

    if (loadCellConnected) {
      float reading1 = scale.get_units(5);
      delay(100);
      reading2 = scale.get_units(5);

      if (abs(reading1 - reading2) < 1.0) {
        lastValidWeight = reading2;
        Serial.print("Weight: ");
        Serial.print(reading2, 2);
        Serial.println(" g");
      }
      reading2 = lastValidWeight;  // siempre publica el ultimo valor confiable
    }

    // --- Cálculo del flujo cada 1 segundo ---
    unsigned long now = millis();
    if (now - lastFlowCalc >= 1000) {
      detachInterrupt(digitalPinToInterrupt(FLOW_PIN));
      unsigned long pulses = pulseCount;
      pulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);

      float elapsedSec = (now - lastFlowCalc) / 1000.0;
      lastFlowCalc = now;

      float litersThisInterval = pulses / PULSES_PER_LITER;
      totalVolume_L += litersThisInterval;

      flowRate_Lmin = (litersThisInterval / elapsedSec) * 60.0;

      Serial.print("Flow: ");
      Serial.print(flowRate_Lmin, 3);
      Serial.print(" L/min | Total: ");
      Serial.print(totalVolume_L, 3);
      Serial.println(" L");
    }

    // Convertir floats a texto
    char mensajeweight[12];
    if (loadCellConnected) {
      dtostrf(reading2, 5, 2, mensajeweight);
    } else {
      strcpy(mensajeweight, "NA");
    }

    char mensajeflow[12];
    dtostrf(flowRate_Lmin, 8, 3, mensajeflow);

    // Armar mensaje combinado: "weight: X flow: Y"
    char mensaje[48];
    snprintf(mensaje, sizeof(mensaje), "weight: %s flow: %s", mensajeweight, mensajeflow);

    // Publicar mensaje combinado
    client.publish(mqtt_topic, mensaje);

    Serial.print("Muestra ");
    Serial.print(contador + 1);
    Serial.print(": ");
    Serial.println(mensaje);

    contador++;
    delay(300);
  } else {
    Serial.println("FIN DE PUBLICACION");
    client.publish(mqtt_topic, "FIN DE PUBLICACION");
    while (true) {
      client.loop();
    }
  }
}