[README_Datalogger_ESP32_Pedro.md](https://github.com/user-attachments/files/22691875/README_Datalogger_ESP32_Pedro.md)
# README – Datalogger IoT ESP32 + YL-83 + RTC + microSD + LCD + MQTT

**Autor:** Pedro Iván Palomino Viera  
**Tema:** Práctica de Datalogger con ESP32 y PCB Casera  

---

## 1. Objetivo General
Diseñar, implementar y validar un datalogger IoT basado en ESP32 que mida humedad/lluvia con sensor **YL-83**, registre datos con sello de tiempo en **microSD**, los publique por **MQTT** y muestre estado en **LCD**, cumpliendo criterios de programación no bloqueante y documentación técnica.

---

## 2. Objetivos Específicos
- Configurar conectividad Wi-Fi y enlace MQTT hacia el broker público Mosquitto.  
- Adquirir la señal del sensor YL-83 y aplicar umbrales con histéresis para accionar un buzzer.  
- Registrar periódicamente datos en formato JSON con fecha y hora (RTC DS1307) en archivos diarios en microSD.  
- Mostrar en LCD I2C el estado del sistema, valor de sensor y alertas del actuador.  
- Estructurar el firmware con tareas no bloqueantes usando `millis()`, clases y métodos reutilizables.  
- Publicar telemetría JSON en el topic MQTT `20194989_Pedro` y verificar recepción.  
- Documentar conexiones hardware, parámetros configurables y pasos de despliegue.  

---

## 3. Competencias
- Integración de hardware IoT (sensores, almacenamiento, visualización).  
- Programación embebida no bloqueante en ESP32 (C++/Arduino).  
- Diseño y depuración de buses SPI e I2C.  
- Serialización de datos (JSON) y mensajería MQTT (publish/subscribe).  
- Gestión de versiones y documentación técnica.  

---

## 4. Tabla de Contenidos
1. [Objetivo General](#1-objetivo-general)  
2. [Objetivos Específicos](#2-objetivos-específicos)  
3. [Competencias](#3-competencias)  
4. [Tabla de Contenidos](#4-tabla-de-contenidos)  
5. [Descripción](#5-descripción)  
6. [Requisitos](#6-requisitos)  
7. [Instalación y Configuración](#7-instalación-y-configuración)  
8. [Conexiones de Hardware](#8-conexiones-de-hardware)  
9. [Uso y ejemplos de Código](#9-uso-y-ejemplos-de-código)  
10. [Resultados de Prueba](#10-resultados-de-prueba)  
11. [Consideraciones Éticas y de Seguridad](#11-consideraciones-éticas-y-de-seguridad)  
12. [Solución de Problemas](#12-solución-de-problemas)  
13. [Contribuciones](#13-contribuciones)  
14. [Referencias](#14-referencias)  

---

## 5. Descripción
Este proyecto integra un datalogger para entornos inteligentes usando **ESP32**. El sistema mide lluvia/humedad con **YL-83**, registra lecturas con fecha/hora del **RTC DS1307**, almacena datos en **microSD** y los publica en un broker **MQTT** para monitoreo remoto.  
La interfaz **LCD I2C** y el **LED RGB** indican el estado de red y del actuador. La arquitectura demuestra prácticas IoT: comunicación, almacenamiento local y ejecución no bloqueante.

---

## 6. Requisitos

### Hardware necesario
- ESP32 DevKit (Wi-Fi y ADC).  
- Sensor de lluvia YL-83 (AO).  
- RTC DS1307 con cristal y batería.  
- Módulo microSD (socket a 5 V) sobre SPI (CS, MOSI, MISO, SCK).  
- LCD 16x2 I2C.  
- Buzzer activo.  
- LED RGB (3 pines) + resistencias.  
- Protoboard/PCB, cables, fuente de 5 V estable.  

### Software y bibliotecas requeridas
- Arduino IDE + ESP32 core.  
- Librerías: `WiFi.h`, `PubSubClient`, `RTClib`, `LiquidCrystal_I2C`, `SPI`, `SD`, `ArduinoJson`.  
- Broker MQTT: `test.mosquitto.org` (puerto 1883).  

### Conocimientos previos imprescindibles
- C/C++ para Arduino/ESP32 (clases, `millis()`).  
- Buses SPI e I2C, lectura ADC.  
- JSON y MQTT (topics, QoS básico).  
- Manejo de microSD (FAT32) y depuración eléctrica.  
- Técnicas de planchado y perforado de PCB casera.  

---

## 7. Instalación y Configuración
1. Clonar o descargar el repositorio del proyecto.  
2. Abrir el sketch principal en Arduino IDE.  
3. Instalar las bibliotecas indicadas.  
4. Configurar credenciales Wi-Fi (`WIFI_SSID` y `WIFI_PASS`).  
5. Verificar el topic MQTT `20194989_Pedro` y el host `test.mosquitto.org:1883`.  
6. Ajustar pines de hardware (YL-83, SD, LCD, LED RGB, buzzer).  
7. Compilar y subir al ESP32.  
8. Monitorear el diagnóstico inicial por Serial.  
9. Ajustar umbrales `UMBRAL_ON` y `UMBRAL_OFF` según calibración del sensor.  

---

## 8. Conexiones de Hardware

| Señal del módulo | Pin de la placa | Función |
|------------------|-----------------|----------|
| YL-83 AO | GPIO34 (ADC1) | Entrada analógica de humedad/lluvia |
| Buzzer IN | GPIO25 | Actuador acústico (intermitente) |
| LED RGB R | GPIO15 | Rojo sólido: sin Wi-Fi |
| LED RGB G | GPIO2 | Verde: Wi-Fi + MQTT OK |
| LED RGB B | GPIO4 | Azul parpadeo: conectando |
| LCD I2C SDA | GPIO21 | Bus I2C |
| LCD I2C SCL | GPIO22 | Bus I2C |
| RTC DS1307 SDA | GPIO21 | Bus I2C compartido |
| RTC DS1307 SCL | GPIO22 | Bus I2C compartido |
| SD CS | GPIO5 | Chip Select SPI |
| SD MOSI (CMD) | GPIO23 | SPI VSPI |
| SD MISO (DAT0) | GPIO19 | SPI VSPI |
| SD SCK (CLK) | GPIO18 | SPI VSPI |
| VDD SD | 3.3 V | Alimentación segura |
| GND | GND común | Referencia |

---

## 9. Uso y ejemplos de Código
El código fuente se organiza dentro de `/datalogger`, dividido en clases: `SensorYL83`, `ActuatorBuzzer`, `DisplayLCD`, `StorageSD`, `MqttClient` y `LedRGB`.  
Cada clase incluye comentarios de bloque que describen su función, parámetros y efectos.  

El programa inicializa todos los módulos en `setup()` y ejecuta tareas periódicas con `millis()` en lugar de `delay()`. Estas tareas leen el sensor, controlan el buzzer, actualizan el LCD, registran datos en la microSD, publican por MQTT y gestionan el LED RGB de estado.  

Los datos se serializan en **JSON** y se guardan en la **microSD** y se publican en **Mosquitto** bajo el topic `20194989_Pedro`. El LCD muestra fecha, hora y lectura del sensor, mientras que el buzzer y el LED reflejan el estado operativo.

---

## 10. Resultados de Prueba
El sistema funcionó de forma estable, cumpliendo todos los objetivos. Se verificó la conexión Wi-Fi y la publicación en el broker MQTT, la creación de archivos diarios JSON en la microSD y la visualización en el LCD.  

El buzzer respondió intermitentemente al detectar humedad, y el LED RGB cambió de color según el estado de conexión. En el monitor serial se visualizaron mensajes de diagnóstico y confirmaciones de escritura en la microSD.  

El archivo JSON generado almacena fecha, hora, valor del sensor y estado del actuador. Las fotografías del montaje y capturas del monitor serial validan la integración completa de hardware y software del datalogger IoT.

---

## 11. Consideraciones Éticas y de Seguridad
- Privacidad de datos: anonimizar identificadores.  
- Seguridad en MQTT: usar topics no triviales y considerar autenticación/TLS.  
- Riesgos eléctricos: operar microSD a 3.3 V y respetar corrientes máximas.  
- Disponibilidad: implementar reconexiones Wi-Fi/MQTT.  
- Cumplimiento: informar finalidad y tiempo de retención de datos.  

---

## 12. Solución de Problemas
- **SD no detectada (CARD_NONE):** verificar VDD = 3.3 V, soldaduras y frecuencia SPI.  
- **LCD sin respuesta:** confirmar dirección I2C (0x27/0x3F) y pines SDA/SCL.  
- **MQTT no conecta:** revisar Wi-Fi, host o puerto.  
- **Buzzer activo en seco:** ajustar umbrales y revisar histéresis.  
- **Lecturas erráticas:** comprobar GND común y desacoples.  

---

## 13. Contribuciones
1. Realizar *fork* del repositorio:  
   👉 [https://github.com/Spartan00734/Esqueleto-para-un-entorno-Inteligente.git](https://github.com/Spartan00734/Esqueleto-para-un-entorno-Inteligente.git)  
2. Crear una rama `feature/mi-mejora`.  
3. Hacer *commits* documentados.  
4. Abrir un *Pull Request* con descripción de cambios y pruebas.  

---

## 14. Referencias
- RTClib (Adafruit) – documentación y ejemplos.  
- PubSubClient – documentación oficial.  
- ArduinoJson – guía de uso.  
- ESP32 Arduino Core – referencia de pines e interfaces.  
- Fichas técnicas de YL-83, DS1307 y módulos microSD.  
- [Repositorio en GitHub](https://github.com/Spartan00734/Esqueleto-para-un-entorno-Inteligente.git)
