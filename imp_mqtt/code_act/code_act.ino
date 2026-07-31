#include "esp_timer.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

#include <Arduino.h>
#include "HX711.h"

// =========================
// ENTRADAS SENSORES
// =========================
#define DT_PIN 4
#define SCK_PIN 5
#define FLOW_PIN 15   // GPIO D15 - YF-S401
#define SERVO_PIN 16  // GPIO D16 - Servo

// =========================
// DATOS DE WIFI
// =========================
const char* ssid = "realme C67";
const char* password = "jkgh23!;";

// =========================
// DATOS DEL BROKER MQTT
// =========================
const char* mqtt_server = "10.235.51.191";  // actualizar 
const int mqtt_port = 1883;
const char* mqtt_topic = "kalman_proj";

// =========================
// SENSOR BALANZA
// =========================
bool loadCellConnected = false;
HX711 scale;
float calibration_factor = 428.473;

// Queue de tamaño 1: siempre contiene el último peso leído.
// Se usa xQueueOverwrite/xQueuePeek en vez de Send/Receive para que
// loop() siempre pueda leer el valor más reciente sin "vaciar" la queue.
QueueHandle_t weightQueue;

// =========================
// SERVO
// =========================
Servo miServo;
volatile int anguloActual = 0;

// Queue para enviar comandos de ángulo a taskServo
QueueHandle_t servoQueue;

// =========================
// OBJETOS WIFI Y MQTT
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

// --- Calibración del sensor de flujo ---
const float PULSOS_POR_LITRO = 1300.0; //

// --- Variables compartidas entre ISR y tarea ---
volatile uint32_t pulseCount = 0;    // contador crudo, tocado por la ISR
volatile float flowRate_Lmin = 0.0;  // resultado, lo lee MQTT
volatile float totalVolume_L = 0.0;  // acumulado total

esp_timer_handle_t flowCalcTimer;

// Spinlock para proteger pulseCount entre la ISR (puede correr en un core)
// y flowCalcCallback (corre en la tarea interna de esp_timer, en el otro
// core). portDISABLE_INTERRUPTS() por si solo NO alcanza porque solo
// afecta al core local; portENTER_CRITICAL / _ISR con este mux sincroniza
// ambos cores correctamente.
static portMUX_TYPE flowMux = portMUX_INITIALIZER_UNLOCKED;

// --- ISR: lo más corta posible ---
void IRAM_ATTR flowPulseISR() {
  portENTER_CRITICAL_ISR(&flowMux);
  pulseCount++;
  portEXIT_CRITICAL_ISR(&flowMux);
}

// --- Callback del timer, se ejecuta cada 1000 ms ---
void flowCalcCallback(void* arg) {
  uint32_t pulses;

  // Sección crítica real (cross-core) usando el mismo spinlock que la ISR
  portENTER_CRITICAL(&flowMux);
  pulses = pulseCount;
  pulseCount = 0;
  portEXIT_CRITICAL(&flowMux);

  // Pulsos en 1 segundo -> litros/segundo -> litros/minuto
  float litrosEsteSegundo = pulses / PULSOS_POR_LITRO;
  flowRate_Lmin = litrosEsteSegundo * 60.0;
  totalVolume_L += litrosEsteSegundo;

  // --- DIAGNOSTICO TEMPORAL: quitar una vez resuelto el problema de los ceros ---
  Serial.print("[flow debug] pulsos_crudos=");
  Serial.print(pulses);
  Serial.print("  flowRate_Lmin=");
  Serial.println(flowRate_Lmin, 3);
}

void setupFlowSensor() {
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowPulseISR, FALLING);

  const esp_timer_create_args_t timerArgs = {
    .callback = &flowCalcCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "flow_calc_timer"
  };

  esp_timer_create(&timerArgs, &flowCalcTimer);
  esp_timer_start_periodic(flowCalcTimer, 1000000);  // 1,000,000 us = 1 s
}


// =========================
// TAREA: CELDA DE CARGA
// =========================
void taskLoadCell(void* param) {
  for (;;) {
    if (loadCellConnected && scale.is_ready()) {
      float reading1 = scale.get_units(5);
      vTaskDelay(pdMS_TO_TICKS(50));
      float reading2 = scale.get_units(5);

      // Solo aceptamos la lectura si es estable (poca diferencia entre tomas)
      if (fabs(reading1 - reading2) < 1000.0) {
        // xQueueOverwrite reemplaza el contenido si ya había un valor;
        // como la queue tiene tamaño 1, siempre queda solo el más reciente.
        xQueueOverwrite(weightQueue, &reading2);
        Serial.print("Weight: ");
        Serial.print(reading2, 2);
        Serial.println(" g");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =========================
// TAREA: SERVO (recibe comandos desde la queue)
// =========================
void taskServo(void* param) {
  int angulo;
  for (;;) {
    if (xQueueReceive(servoQueue, &angulo, portMAX_DELAY)) {
      if (angulo >= 0 && angulo <= 90) {
        miServo.write(angulo);
        anguloActual = angulo;
      }
    }
  }
}

// =========================
// TAREA: LECTURA DE SERIAL PARA EL SERVO
// =========================
void taskSerialInput(void* param) {
  for (;;) {
    if (Serial.available() > 0) {
      int angulo = Serial.parseInt();
      // Descartar el resto del buffer (salto de línea, etc.)
      while (Serial.available() > 0) Serial.read();

      if (angulo >= 0 && angulo <= 90) {
        xQueueSend(servoQueue, &angulo, portMAX_DELAY);
      } else {
        Serial.println("Angulo fuera de rango (0-90).");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

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

void setup() {
  Serial.begin(115200);
  delay(2000);

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Inicio de adquisicion continua por MQTT");

  // Servo
  miServo.setPeriodHertz(50);
  miServo.attach(SERVO_PIN, 500, 2400);
  miServo.write(anguloActual);
  Serial.println("Escribe un angulo entre 0 y 90 (Enter) para mover el servo:");

  // Celda de carga
  scale.begin(DT_PIN, SCK_PIN);
  Serial.println("Checking load cell...");
  if (scale.wait_ready_timeout(200)) {
    loadCellConnected = true;
    scale.set_scale(calibration_factor);
    Serial.println("Taring...");
    scale.tare();
    Serial.println("Tare complete.");
  } else {
    Serial.println("Load cell NOT detected. Reinicia la ESP32 despues de conectarla.");
  }

  // Queue para comandos del servo
  servoQueue = xQueueCreate(5, sizeof(int));

  // Queue de tamaño 1 para el peso (siempre guarda el último valor)
  weightQueue = xQueueCreate(1, sizeof(float));

  // Tareas paralelas (sensores)
  setupFlowSensor();
  xTaskCreatePinnedToCore(taskLoadCell, "loadcell", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskServo, "servo", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSerialInput, "serialInput", 2048, NULL, 1, NULL, 1);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Convertir floats a texto
  char mensajePeso[12];
  float pesoActual = 0.0;
  // xQueuePeek NO retira el dato de la queue, solo lo copia; así el valor
  // sigue disponible para la próxima publicación aunque taskLoadCell no
  // haya escrito uno nuevo todavía. timeout 0 = no bloquear si aún no hay dato.
  if (loadCellConnected && xQueuePeek(weightQueue, &pesoActual, 0) == pdTRUE) {
    dtostrf(pesoActual, 5, 2, mensajePeso);
  } else {
    strcpy(mensajePeso, "NA");
  }

  char mensajeFlow[12];
  dtostrf(flowRate_Lmin, 8, 3, mensajeFlow);

  char mensajeVolumen[12];
  dtostrf(totalVolume_L, 8, 3, mensajeVolumen);

  // Armar mensaje combinado
  char mensaje[96];
  snprintf(mensaje, sizeof(mensaje),
           "angle: %d weight: %s flow: %s vol: %s",
           anguloActual, mensajePeso, mensajeFlow, mensajeVolumen);

  client.publish(mqtt_topic, mensaje);

  vTaskDelay(pdMS_TO_TICKS(1000));
}