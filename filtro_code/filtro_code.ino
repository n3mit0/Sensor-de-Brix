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
const char* mqtt_server = "10.135.51.191";  // actualizar
const int mqtt_port = 1883;
const char* mqtt_topic = "kalman_proj";

// =========================
// SENSOR BALANZA
// =========================
bool loadCellConnected = false;
HX711 scale;
float calibration_factor = 428.473;

// Queue de tamaño 1: siempre contiene el último peso leído.
QueueHandle_t weightQueue;

// =========================
// SERVO
// =========================
Servo miServo;
volatile int anguloActual = 90;

QueueHandle_t servoQueue;

// =========================
// FILTRO DE KALMAN
// =========================
// Estado: x_k = [m_s, m_T]  (masa de azucar, masa total del container)
// Entrada (control, u_k) = caudal medido Q_k (flowRate_Lmin)
// Medicion (observacion, z_k) = masa total medida por la celda de carga (H = [0 1])
float kalman_ms = 0.0;    // estado estimado: masa de azucar
float kalman_mT = 0.0;    // estado estimado: masa total
float kalman_ms_1 = 0.0;  // estado estimado: masa total

// Covarianza del error de estimacion (matriz simetrica 2x2 -> 3 valores)
// P0 = I es razonable si no hay certeza inicial sobre m_s(0), m_T(0).
float kalmanP11 = 1.0;
float kalmanP12 = 0.0;
float kalmanP22 = 1.0;

// Ruido de proceso (matriz Q_proc, simetrica 2x2)
// TODO: definir segun que tan bien confias en el modelo dinamico (F, B, G).
// Valores mas altos = el filtro confia menos en la prediccion y reacciona
// mas rapido a la medicion.
float kalmanQ11 = 0.001;  // pequeño: confías en el modelo de azúcar
float kalmanQ22 = 0.06;   // más grande: no confías en el modelo de m_T
float kalmanQ12 = 0.0;    // empieza en 0, sin covarianza cruzada asumida

// Ruido de medicion (matriz R, simetrica 2x2, diagonal si los sensores
// son independientes entre si)
// R1: varianza del sensor de flujo (z1)
// R2: varianza de la celda de carga (z2)
// TODO: definir a partir de la varianza real de cada sensor
// (ej. tomar N lecturas en reposo/flujo constante y calcular su varianza).
// OJO: dejar esto en 0 puede producir division por cero en det(Sk) si
// la covarianza asociada tambien colapsa a 0.
float kalmanR1 = 0.05;      // desviacion^2 estimada del sensor de flujo
float kalmanR2 = 0.001719;  // desviacion^2 de la celda de carga

// Parametros fisicos del modelo dinamico
// TODO: definir con datos reales del sistema.
float rho = 1.06 * 1000.0;  // densidad del fluido de entrada [kg/L o unidades consistentes con Q]
float x1 = 17.0 / 100.0;    // concentracion masa/masa en la entrada (Ball Valve), 0..1 brix/100 g
//float E = 0.41202947;     // termino de perdida/sesgo en la prediccion de mT (B = [0; -E])
float E = 0.55;             // kg/min tasa de evaporación

// Debe coincidir con el periodo del timer del Kalman (setupKalmanTimer)
const float KALMAN_DT = 0.1;  // 100 ms

// Pequeno valor para evitar division por cero si det(Sk) colapsa a 0
// (puede pasar si algun R = 0 y el filtro ya convergio con P -> 0)
const float KALMAN_EPS = 1e-6;

// Queue de tamaño 1: guarda el resultado final (m_s / m_T) para que
// taskMQTT lo lea y lo publique/imprima.
QueueHandle_t kalmanQueue;

esp_timer_handle_t kalmanTimer;
TaskHandle_t kalmanTaskHandle = NULL;

// =========================
// OBJETOS WIFI Y MQTT
// =========================
WiFiClient espClient;
PubSubClient client(espClient);

// --- Calibración del sensor de flujo ---
// YF-S401: 98 Hz por cada L/min de caudal -> 98*60 = 5880 pulsos/litro
const float PULSOS_POR_LITRO = 1300.0;

// --- Variables compartidas entre ISR y tareas ---
volatile uint32_t pulseCount = 0;    // contador crudo, tocado por la ISR
volatile float flowRate_Lmin = 0.0;  // resultado, lo lee taskKalman y taskMQTT
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
  // Serial.print("[flow debug] pulsos_crudos=");
  // Serial.print(pulses);
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

float pwm_Q(int pwm) {  // lo devuelve en L/min
  if (pwm != 90) {
    if (pwm < 80) {
      return 0.738;
    } else {
      return -0.0149 * pwm * pwm + 2.4109 * pwm - 96.422;
    }
  } else {
    return 0;
  }
}

// --- Callback del timer del Kalman: SOLO notifica, no calcula nada aca ---
void kalmanTimerCallback(void* arg) {
  if (kalmanTaskHandle != NULL) {
    xTaskNotifyGive(kalmanTaskHandle);
  }
}

void setupKalmanTimer() {
  const esp_timer_create_args_t timerArgs = {
    .callback = &kalmanTimerCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "kalman_timer"
  };

  esp_timer_create(&timerArgs, &kalmanTimer);
  esp_timer_start_periodic(kalmanTimer, 100000);  // 100 ms == KALMAN_DT
}

// =========================
// TAREA: FILTRO DE KALMAN
// =========================
// --- Agregar arriba, junto a las demás declaraciones globales ---
// float kalmanR1 = ...;             // varianza ruido sensor de flujo (z1)
// float kalmanR2 = ...;             // varianza ruido celda de carga (z2)
// extern QueueHandle_t flowSensorQueue;  // cola del sensor de flujo (ajustar nombre si difiere)

void taskKalman(void* param) {
  for (;;) {
    // Se queda bloqueada (sin gastar CPU) hasta que el timer la notifique
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // -----------------------------------------------------
    // Entrada de control (u_k): PWM -> flujo
    // -----------------------------------------------------
    int pwm = anguloActual;
    float uk = pwm_Q(pwm) * (1.0 / 60.0);  // verificar que quede en L/s

    // -----------------------------------------------------
    // Mediciones (z_k): sensor de flujo (z1) y celda de carga (z2)
    // -----------------------------------------------------
    // float z1 = flowRate_Lmin*(1.0/60.0); // se pasa a L/seg
    float z1 = rho * x1 * totalVolume_L;  // g masa de azucar acumulada, segun el volumen acumulado por el sensor de flujo
    float z2 = 0.0;
    bool hayMasa = (xQueuePeek(weightQueue, &z2, 0) == pdTRUE);

    if (!hayMasa) {
      continue;  // todavia no hay lectura de ambos sensores
    }

    // ---------------------------------------------------------
    // 1) PREDICCION
    //    F = I  =>  suma directa de estados
    //    G = [rho*x1*dt ; rho*dt]   B = [0 ; -E]
    // ---------------------------------------------------------
    float ms_pred = kalman_ms + rho * x1 * KALMAN_DT * uk;
    float mT_pred = kalman_mT + rho * KALMAN_DT * uk - E * KALMAN_DT * (1000.0 / 60.0); // E en gramos

    float P11_pred = kalmanP11 + kalmanQ11;
    float P12_pred = kalmanP12 + kalmanQ12;
    float P22_pred = kalmanP22 + kalmanQ22;

    // ---------------------------------------------------------
    // 2) INNOVACION
    //    H = [1, 0 ; 0, 1/(rho*x1)] pero como Z2 ya es masa despues de medir
    // ---------------------------------------------------------

    float y1 = z1 - ms_pred;
    float y2 = z2 - mT_pred;

    // S = H * P_pred * H^T + R  (2x2, R diagonal)
    float S11 = P11_pred + kalmanR1;
    float S12 = P12_pred;
    float S21 = S12;
    float S22 = P22_pred + kalmanR2;

    float det = S11 * S22 - S12 * S21;
    if (fabs(det) < KALMAN_EPS) det = (det >= 0 ? KALMAN_EPS : -KALMAN_EPS);
    float invDet = 1.0 / det;

    float Si11 = S22 * invDet;
    float Si12 = -S12 * invDet;
    float Si21 = -S21 * invDet;
    float Si22 = S11 * invDet;

    // M = P_pred * H^T
    float M11 = P11_pred;
    float M12 = P12_pred;
    float M21 = P12_pred;
    float M22 = P22_pred;

    // K = M * S^-1
    float K11 = M11 * Si11 + M12 * Si21;
    float K12 = M11 * Si12 + M12 * Si22;
    float K21 = M21 * Si11 + M22 * Si21;
    float K22 = M21 * Si12 + M22 * Si22;

    // ---------------------------------------------------------
    // 3) CORRECCION
    // ---------------------------------------------------------
    kalman_ms = ms_pred + K11 * y1 + K12 * y2;
    kalman_mT = mT_pred + K21 * y1 + K22 * y2;

    // P_upd = (I - K*H) * P_pred
    float KH11 = K11;
    float KH12 = K12;
    float KH21 = K21;
    float KH22 = K22;

    kalmanP11 = (1 - KH11) * P11_pred - KH12 * P12_pred;
    kalmanP12 = (1 - KH11) * P12_pred - KH12 * P22_pred;
    kalmanP22 = -KH21 * P12_pred + (1 - KH22) * P22_pred;

    // ---------------------------------------------------------
    // 4) SALIDA: fraccion masica de azucar (Brix = x*100%)
    // ---------------------------------------------------------
    float resultadoFinal = 0.0;
    //Serial.print("ms: ");
    //Serial.print(kalman_ms);
    //Serial.print(",  mT:");
    //Serial.println(kalman_mT);
    if (kalman_mT > 0.0001) {
      resultadoFinal = (kalman_ms / kalman_mT) * 100.0;
    }
    xQueueOverwrite(kalmanQueue, &resultadoFinal);
  }
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

      if (fabs(reading1 - reading2) < 1000.0) {
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
// TAREA: SERVO
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
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// =========================
// TAREA NUCLEO 0: WIFI + MQTT + PUBLICACION
// (antes vivia en loop(), que por defecto corre en el nucleo 1;
//  se movio aca para no competir con las tareas de sensores)
// =========================
void taskMQTT(void* param) {
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  Serial.println("Inicio de adquisicion continua por MQTT");

  for (;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    char mensajePeso[12];
    float pesoActual = 0.0;
    if (loadCellConnected && xQueuePeek(weightQueue, &pesoActual, 0) == pdTRUE) {
      dtostrf(pesoActual, 5, 2, mensajePeso);
    } else {
      strcpy(mensajePeso, "NA");
    }

    char mensajeFlow[12];
    dtostrf(flowRate_Lmin, 8, 3, mensajeFlow);

    char mensajeVolumen[12];
    dtostrf(totalVolume_L, 8, 3, mensajeVolumen);

    char mensajeKalman[12];
    float kalmanResultado = 0.0;
    if (xQueuePeek(kalmanQueue, &kalmanResultado, 0) == pdTRUE) {
      dtostrf(kalmanResultado, 8, 3, mensajeKalman);
    } else {
      strcpy(mensajeKalman, "NA");
    }

    char mensaje[128];
    snprintf(mensaje, sizeof(mensaje),
             "angle: %d °brix: %s",
             anguloActual, mensajeKalman);

    Serial.println(mensaje);
    client.publish(mqtt_topic, mensaje);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Escribe un angulo entre 0 y 90 (Enter) para mover el servo:");

  // Servo
  miServo.setPeriodHertz(50);
  miServo.attach(SERVO_PIN, 500, 2400);
  miServo.write(anguloActual);

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

  // Queues
  servoQueue = xQueueCreate(5, sizeof(int));
  weightQueue = xQueueCreate(1, sizeof(float));
  kalmanQueue = xQueueCreate(1, sizeof(float));

  // Sensor de flujo (ISR + esp_timer)
  setupFlowSensor();

  // Tareas del nucleo 1: sensores y actuador
  xTaskCreatePinnedToCore(taskLoadCell, "loadcell", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskServo, "servo", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSerialInput, "serialInput", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskKalman, "kalman", 4096, NULL, 2, &kalmanTaskHandle, 1);
  setupKalmanTimer();

  // Tarea del nucleo 0: WiFi + MQTT
  xTaskCreatePinnedToCore(taskMQTT, "mqtt_task", 6144, NULL, 1, NULL, 0);
}

// loop() queda vacio: todo el trabajo vive en las tareas de FreeRTOS.
void loop() {
  vTaskDelete(NULL);
}