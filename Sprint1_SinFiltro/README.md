# Sprint 1 - Sistema de Tele-operación de Brazo Robótico 2DOF

## 📋 Resumen Ejecutivo

Sistema de control remoto para brazo robótico de 2 grados de libertad mediante IMU MPU6050 y comunicación inalámbrica ESP-NOW. Este Sprint 1 establece la **línea base sin filtros** para comparación con futuros sprints que implementarán filtrado digital.

**Estado:** ✅ Completado y Funcional

---

## 🎯 Características Principales

- ⚡ **Comunicación ESP-NOW:** Latencia ~20ms, >99% de éxito
- 🎮 **Control Gestual:** Movimiento de mano controla servos mediante IMU
- 🔧 **Hardware Dual ESP32:** WROOM (transmisor) + ESP32-S3 (receptor)
- 📊 **Salida DAC:** Señal analógica para análisis de espectro
- 🛡️ **Seguridad:** Timeout automático y suavizado incremental
- 📡 **50Hz de muestreo:** Respuesta fluida en tiempo real

---

## 📂 Estructura del Repositorio

```
Sprint1_SinFiltro/
│
├── 📄 README.md                    ← Este archivo (índice general)
├── 📄 Sprint1.md                   ← Documentación técnica completa
│
├── 📁 Transmisor_Guante/           ← Código ESP32 WROOM (guante)
│   └── Transmisor_Guante.ino
│
├── 📁 Receptor_Brazo/              ← Código ESP32-S3 (brazo)
│   └── Receptor_Brazo.ino
│
└── 🛠️ Herramientas de Diagnóstico:
    ├── I2C_Scanner_Auto/           ← Detectar dispositivos I2C
    ├── Obtener_MAC_WROOM/          ← Ver MAC ESP32 WROOM
    └── Obtener_MAC_S3/             ← Ver MAC ESP32-S3
```

---

## 🚀 Inicio Rápido

### 1️⃣ Hardware Requerido

| Componente | Cantidad | Notas |
|------------|----------|-------|
| ESP32 WROOM | 1 | Transmisor (tiene DAC) |
| ESP32-S3 | 1 | Receptor (controla servos) |
| MPU6050 | 1-2 | 1 obligatorio (guante), 1 opcional (brazo) |
| Servos MG90S | 2 | Alimentación externa 5V/2A |
| Fuente 5V/2A | 1 | Para servos |

### 2️⃣ Conexiones

**ESP32 WROOM (Transmisor/Guante):**
```
MPU6050:  SDA → GPIO21  |  SCL → GPIO22
DAC Out:  Señal → GPIO25 (para osciloscopio/analizador)
```

**ESP32-S3 (Receptor/Brazo):**
```
MPU6500:  SDA → GPIO8   |  SCL → GPIO10  (opcional)
Servo 1:  Base → GPIO6   |  5V + GND externa
Servo 2:  Efector → GPIO7  |  5V + GND externa
```

### 3️⃣ Configuración Software

**Arduino IDE:**
- Board Manager: ESP32 v2.0.11+
- Librerías: `ESP32Servo`, `Adafruit MPU6050`, `Adafruit Unified Sensor`
- **IMPORTANTE para ESP32-S3:** Tools → USB CDC On Boot → **Enabled**

### 4️⃣ Carga de Código

1. **Transmisor:** Cargar `Transmisor_Guante/Transmisor_Guante.ino` en ESP32 WROOM
2. **Receptor:** Cargar `Receptor_Brazo/Receptor_Brazo.ino` en ESP32-S3
3. **Verificación:** Ambos dispositivos deben mostrar mensajes en Serial Monitor

---

## 📊 Resultados de Pruebas

| Métrica | Valor | Estado |
|---------|-------|--------|
| Latencia promedio | ~20ms | ✅ Excelente |
| Frecuencia muestreo | 50Hz | ✅ Estable |
| Éxito ESP-NOW | >99% | ✅ Confiable |
| Tremor (sin filtro) | ±5° | ✅ Esperado (baseline) |
| Tiempo respuesta servo | <100ms | ✅ Fluido |

> **Nota:** El tremor de ±5° es intencional en Sprint 1 (sin filtros). Será reducido en Sprints 2 y 3.

---

## 🔍 Documentación Completa

Para detalles técnicos exhaustivos, incluyendo:
- Arquitectura del sistema y diagramas de conexión
- Algoritmos de control y suavizado incremental
- Protocolos de comunicación ESP-NOW
- Análisis de problemas resueltos
- Validación experimental y resultados

**👉 Consultar: [`Sprint1.md`](Sprint1.md)**

---

## 🛠️ Herramientas de Diagnóstico

### I2C Scanner
```bash
Cargar: I2C_Scanner_Auto/I2C_Scanner_Auto.ino
Uso: Detecta direcciones I2C de MPU6050/MPU6500
```

### Obtener MAC Address
```bash
ESP32 WROOM: Obtener_MAC_WROOM/Obtener_MAC_WROOM.ino
ESP32-S3:    Obtener_MAC_S3/Obtener_MAC_S3.ino
```

---

## 🔧 Solución de Problemas Comunes

| Problema | Solución |
|----------|----------|
| ESP32-S3 no muestra Serial | Tools → USB CDC On Boot → **Enabled** |
| MAC address 00:00:00:00:00:00 | Agregar `delay(100)` después de `WiFi.mode()` |
| Servos tiemblan mucho | Normal en Sprint 1 (sin filtros) |
| MPU6050 no detectado | Verificar conexiones I2C, probar I2C Scanner |
| WHO_AM_I = 0x70 | Es MPU6500, totalmente compatible |

---

## 📈 Roadmap

- ✅ **Sprint 1:** Sistema base sin filtros (COMPLETADO)
- ⏳ **Sprint 2:** Filtro de media móvil (pendiente)
- ⏳ **Sprint 3:** Filtro de Kalman + fusión sensorial (pendiente)

---

## 👥 Autores

Universidad Militar Nueva Granada  
Semestre VI - Procesamiento de Señales  
Laboratorio 3 - Sistemas de Control Biomédico

---

## 📜 Licencia

Este proyecto es material académico desarrollado para fines educativos.

---

## 📧 Contacto y Soporte

Para reportar problemas o consultas técnicas, consultar primero la documentación completa en `Sprint1.md`.

---

**🎉 ¡Repositorio listo para producción!**

*Última actualización: Sprint 1 completado y validado*
