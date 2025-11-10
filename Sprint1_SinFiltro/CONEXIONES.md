# Diagrama de Conexiones - Sprint 1

## 🔌 Sistema Completo

```
┌─────────────────────────────────────┐
│      GUANTE (Transmisor)            │
│                                     │
│   ┌─────────────────────────────┐   │
│   │     ESP32 WROOM             │   │
│   │     (Tiene DAC)             │   │
│   │                             │   │
│   │  GPIO 21 ◄────┐            │   │
│   │  GPIO 22 ◄────┤            │   │
│   │  GPIO 25 ────►│ DAC        │   │
│   │  GPIO 2  ────►│ LED        │   │
│   │  3.3V    ────►│            │   │
│   │  GND     ────►│            │   │
│   └───────┬─────────────────────┘   │
│           │                         │
│           │ ESP-NOW                 │
│           │ (2.4GHz)                │
│   ┌───────┴─────────┐               │
│   │    MPU6050      │               │
│   │                 │               │
│   │  SDA ────────────┐             │
│   │  SCL ────────────┘             │
│   │  VCC ── 3.3V                   │
│   │  GND ── GND                    │
│   └─────────────────┘               │
└─────────────────────────────────────┘
              │
              │ Transmisión
              │ Broadcast
              │ 0xFF:FF:FF:FF:FF:FF
              ▼
┌─────────────────────────────────────┐
│   BRAZO ROBÓTICO (Receptor)         │
│                                     │
│   ┌─────────────────────────────┐   │
│   │    ESP32-S3                 │   │
│   │    (NO tiene DAC)           │   │
│   │                             │   │
│   │  GPIO 12 ────►│            │   │
│   │  GPIO 13 ────►│            │   │
│   │  GPIO 48 ────►│ LED        │   │
│   │  5V (ext)────►│            │   │
│   │  GND     ────►│            │   │
│   └─────┬──────┬──────────────┘   │
│         │      │                   │
│         │      │                   │
│   ┌─────▼──┐ ┌─▼─────┐            │
│   │ Servo1 │ │ Servo2 │            │
│   │ (Base) │ │(Extremo)│           │
│   │        │ │        │            │
│   │ MG90S  │ │ MG90S  │            │
│   └────────┘ └────────┘            │
│                                     │
│   Fuente 5V/2A externa              │
└─────────────────────────────────────┘
```

---

## 📊 Pinout ESP32 WROOM (Transmisor - Guante)

```
                   ESP32 WROOM
         ┌────────────────────────┐
         │                        │
    3.3V │ [3V3]          [GND]   │ GND
         │                        │
LED Int  │ [2]  ──► LED           │
         │                        │
I2C SDA  │ [21] ◄── MPU6050 SDA   │
I2C SCL  │ [22] ◄── MPU6050 SCL   │
         │                        │
DAC Out  │ [25] ──► Osciloscopio  │ ✅ DAC1
DAC Out  │ [26] (no usado)        │ ✅ DAC2
         │                        │
         └────────────────────────┘

✅ ESP32 WROOM tiene DAC - Por eso va en el guante
```

---

## 📊 Pinout ESP32-S3 (Receptor - Brazo Robótico)

```
                    ESP32-S3
         ┌────────────────────────┐
         │                        │
    3.3V │ [3V3]          [GND]   │ GND
         │                        │
USB D-   │ [19]  ⚠️ NO USAR       │
USB D+   │ [20]  ⚠️ NO USAR       │
         │                        │
I2C SDA  │ [8]  (opcional IMU)    │
I2C SCL  │ [10] ⚠️ NO GPIO 9      │
         │                        │
Servo1   │ [12] ──► Servo Base    │
Servo2   │ [13] ──► Servo Extremo │
         │                        │
LED Int  │ [48] ──► LED           │
         │                        │
         └────────────────────────┘

❌ ESP32-S3 NO tiene DAC
⚠️ GPIO 9 tiene problemas - usar GPIO 10
```

---

## 🔧 Conexión Servomotores MG90S

```
Vista Superior del Servo MG90S:
┌─────────────────┐
│                 │
│    ┌───┐        │
│    │ ● │ Eje    │
│    └───┘        │
│                 │
│  [Naranja]      │ ◄── Señal PWM (GPIO)
│  [Rojo]         │ ◄── VCC (5V externa)
│  [Marrón]       │ ◄── GND (común)
└─────────────────┘

Conexión:
  Servo1          Servo2
  ------          ------
Naranja → GPIO13  GPIO12
Rojo    → +5V     +5V   (fuente externa)
Marrón  → GND     GND   (común)
```

---

## 🔌 Conexión MPU6050

```
Vista Superior MPU6050:
┌─────────────────┐
│   MPU6050       │
│                 │
│ [VCC] ──► 3.3V  │
│ [GND] ──► GND   │
│ [SCL] ──► GPIO9 │
│ [SDA] ──► GPIO8 │
│ [XDA]           │ (no conectar)
│ [XCL]           │ (no conectar)
│ [AD0]           │ (no conectar)
│ [INT]           │ (no conectar)
└─────────────────┘
```

---

## 📡 Comunicación ESP-NOW

```
Transmisor (ESP32 WROOM)           Receptor (ESP32-S3)
┌──────────────────┐            ┌──────────────────┐
│   GUANTE         │            │   BRAZO          │
│                  │            │                  │
│  WiFi.mode()     │            │  WiFi.mode()     │
│  WIFI_STA        │            │  WIFI_STA        │
│                  │            │                  │
│  esp_now_init()  │            │  esp_now_init()  │
│                  │            │                  │
│  Broadcast:      │  ────────► │  Recibe en:      │
│  FF:FF:FF:FF:FF  │  50 Hz     │  OnDataRecv()    │
│                  │            │                  │
│  Envía:          │            │  Procesa:        │
│  - accelX/Y/Z    │            │  - Control servo │
│  - gyroX/Y/Z     │            │  - Mapeo ángulo  │
│  - handPosition  │            │  - Selección     │
│  - timestamp     │            │                  │
│                  │            │                  │
│  + DAC Output    │            │                  │
│    GPIO25        │            │                  │
└──────────────────┘            └──────────────────┘
```

---

## ⚡ Diagrama de Alimentación

```
┌─────────────────────────────────────┐
│   FUENTE 5V / 2A mínimo             │
└──────────┬──────────────────────────┘
           │
           ├──► Servo1 (VCC)
           │
           ├──► Servo2 (VCC)
           │
           └──► ESP32 (VIN)
           
⚠️ NO alimentar servos desde ESP32 3.3V
⚠️ Consumo por servo: ~500mA en movimiento
⚠️ Total requerido: >2A
```

---

## 🔍 Conexión DAC para Análisis

```
Osciloscopio / Analizador
┌──────────────────┐
│                  │
│  CH1 ◄───────────┼──── GPIO25 (DAC1) ESP32 WROOM
│                  │
│  GND ◄───────────┼──── GND
│                  │
└──────────────────┘

Configuración del Osciloscopio:
- Escala: 1V/div
- Tiempo: 20ms/div
- Acoplamiento: DC
- Trigger: Auto

⚠️ IMPORTANTE: 
- Solo ESP32 WROOM tiene DAC
- ESP32-S3 NO tiene DAC
- Por eso el guante usa ESP32 WROOM
```

---

## 🎯 Lógica de Control Visual

```
Posición de la Mano:

┌─────────────────────────────────────┐
│    MANO ARRIBA (AccelZ > 8 m/s²)    │
│            ┌───┐                    │
│            │ ✋ │ apuntando al cielo  │
│            └───┘                    │
│                                     │
│    ┌──────────────────────────┐    │
│    │  Activa: SERVO 2         │    │
│    │  (Extremo del brazo)     │    │
│    └──────────────────────────┘    │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│    MANO ABAJO (AccelZ < 2 m/s²)     │
│                                     │
│            ┌───┐                    │
│            │ ✋ │ boca abajo         │
│            └───┘                    │
│                                     │
│    ┌──────────────────────────────┐│
│    │  Activa: SERVO 1         │    │
│    │  (Base del brazo)        │    │
│    └──────────────────────────┘    │
└─────────────────────────────────────┘
```

---

## 🔄 Flujo de Datos

```
1. CAPTURA (50 Hz)
   MPU6050 → ESP32-S3
   ↓
   - Leer acelerómetro
   - Leer giroscopio
   - Calcular handPosition

2. TRANSMISIÓN
   ESP32-S3 → ESP-NOW → Broadcast
   ↓
   Paquete de datos (36 bytes)

3. RECEPCIÓN
   ESP32 WROOM ← ESP-NOW
   ↓
   - Callback OnDataRecv()
   - Actualizar variables

4. CONTROL
   ↓
   - Mapear accelX → ángulo
   - Seleccionar servo activo
   - Escribir PWM

5. ACTUACIÓN
   ↓
   Servo1 o Servo2 → Movimiento
```

---

## 📐 Espacio de Trabajo

```
Vista Superior del Brazo:

         Y (adelante)
         ↑
         │
         │     ┌─────┐ Servo2
         │     │  ●  │ (Extremo)
         │     └─────┘
         │        │
         │        │ Brazo Segmento 2
         │        │
         │     ┌─────┐ Servo1
         │     │  ●  │ (Base)
─────────┼─────└─────┘───────► X (derecha)
         │    Base fija
         │
         │
```

**Rango de movimiento**: Plano horizontal (XY)  
**Grados de libertad**: 2 (rotación base + rotación extremo)

---

**Sprint 1** - Versión Sin Filtros  
Noviembre 2025
