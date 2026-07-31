# Sensor de Brix

Este repositorio contiene el firmware y la lógica de control de un sistema embebido basado en ESP32 para el monitoreo y control de flujo en un prototipo de planta. El sistema integra la lectura de una celda de carga, el procesamiento de señales mediante un filtro de Kalman para reducir el ruido de los sensores, y el control de una servoválvula para la regulación del flujo.

Este repositorio es un anexo del proyecto y documenta el proceso de desarrollo del firmware, desde las pruebas individuales de cada sensor hasta la versión final integrada.

## Estructura del repositorio

El código está organizado siguiendo el orden en que se fue desarrollando e integrando el sistema:

### `prueba_calibracion_sensores/`
Contiene los códigos de prueba iniciales, utilizados para verificar el funcionamiento y calibrar cada sensor de forma individual antes de integrarlos al sistema completo:

- **`bal_flujo/`** — Prueba de la balanza (celda de carga) y el sensor de flujo en conjunto.
- **`calibr_bal/`** — Código de calibración de la celda de carga.
- **`medir_bal/`** — Prueba de lectura y medición de la balanza.

### `imp_mqtt/`
Contiene la implementación progresiva de la comunicación MQTT, agregando componentes de forma incremental:

- **`mqtt_demas/`** — Primera integración de los sensores con la comunicación MQTT.
- **`mqtt_demas_servo/`** — Se añade el control del servomotor al sistema con MQTT ya implementado.
- **`code_act/`** — Actualización de la distribución de tareas entre los dos núcleos de la ESP32, además de corrección de errores en la lectura del sensor de flujo, implementando su lectura mediante interrupciones.

### `filtro_code/`
Contiene la **versión final** del firmware (`filtro_code.ino`), que integra todos los desarrollos anteriores:

- Lectura de todos los sensores (celda de carga y demás).
- Comunicación MQTT.
- Control del servomotor / servoválvula.
- Filtro de Kalman para el procesamiento y reducción de ruido en las señales de los sensores y en consecuencia, el cálculo de los grados brix.

Esta es la versión utilizada actualmente en el prototipo.

## Orden de desarrollo

1. Pruebas y calibración individual de sensores (`prueba_calibracion_sensores/`)
2. Integración de sensores con MQTT, agregando el servo progresivamente (`imp_mqtt/`)
3. Versión final con filtro de Kalman, MQTT y control del servo integrados (`filtro_code/`)
