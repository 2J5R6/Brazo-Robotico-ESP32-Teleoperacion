# Sprint 1: Sistema de Teleoperación de Brazo Robótico 2DOF

**Proyecto**: Control de Brazo Robótico mediante IMU  
**Sprint**: 1 - Línea Base Sin Filtros  
**Fecha**: Noviembre 2025  
**Hardware**: ESP32 WROOM + ESP32-S3 + MPU6050 + Servomotores MG90S

---

## 📋 Índice

1. [Resumen Ejecutivo](#resumen-ejecutivo)
2. [Objetivos del Sprint](#objetivos-del-sprint)
3. [Arquitectura del Sistema](#arquitectura-del-sistema)
4. [Especificaciones Técnicas](#especificaciones-técnicas)
5. [Diagramas de Conexión](#diagramas-de-conexión)
6. [Implementación](#implementación)
7. [Resultados y Validación](#resultados-y-validación)
8. [Conclusiones](#conclusiones)

---

## 🎯 Resumen Ejecutivo

El Sprint 1 establece la **línea base** del sistema de teleoperación, implementando comunicación inalámbrica ESP-NOW entre un guante instrumentado (transmisor) y un brazo robótico de 2 grados de libertad (receptor). 

### Logros Principales
- ✅ Comunicación ESP-NOW estable a 50Hz
- ✅ Control de 2 servomotores mediante gestos de la mano
- ✅ Salida DAC para análisis de señales
- ✅ Lógica de selección de servo según orientación de la mano
- ✅ Sistema funcional sin filtrado (baseline para comparación futura)

### Características Clave
- **Latencia**: ~20ms
- **Frecuencia de muestreo**: 50Hz
- **Protocolo**: ESP-NOW broadcast
- **Control**: Proporcional con suavizado incremental
- **Modo sin filtros**: Permite identificar ruido y establecer baseline

---

## 🎯 Objetivos del Sprint

### Objetivo General
Desarrollar un sistema funcional de teleoperación de brazo robótico mediante IMU, estableciendo la arquitectura base y validando la comunicación inalámbrica.

### Objetivos Específicos
1. ✅ Implementar comunicación ESP-NOW entre ESP32 WROOM y ESP32-S3
2. ✅ Leer datos de acelerómetro y giroscopio del MPU6050
3. ✅ Controlar 2 servomotores según orientación de la mano
4. ✅ Generar señal DAC para análisis con osciloscopio
5. ✅ Establecer baseline de ruido (sin filtros)
6. ✅ Implementar lógica de selección de servo
7. ✅ Validar feedback opcional con MPU6050 en el brazo

---

## 🏗️ Arquitectura del Sistema

### Diagrama de Bloques

```
┌──────────────────────────────────────────────────────────────┐
│                    SISTEMA COMPLETO                          │
│                                                              │
│  ┌─────────────────────┐         ┌─────────────────────┐   │
│  │   GUANTE            │         │   BRAZO ROBÓTICO    │   │
│  │   (Transmisor)      │  ESP-   │   (Receptor)        │   │
│  │                     │  NOW    │                     │   │
│  │  ┌──────────────┐   │  ═══►   │  ┌──────────────┐  │   │
│  │  │ MPU6050      │   │  50Hz   │  │ ESP32-S3     │  │   │
│  │  │ Accel+Gyro   │   │         │  │              │  │   │
│  │  └──────┬───────┘   │         │  └──┬───────┬───┘  │   │
│  │         │           │         │     │       │      │   │
│  │  ┌──────▼───────┐   │         │  ┌──▼───┐ ┌▼────┐  │   │
│  │  │ ESP32 WROOM  │   │         │  │Servo1│ │Servo2│ │   │
│  │  │ (con DAC)    │   │         │  │Base  │ │Extr. │ │   │
│  │  │              │   │         │  └──────┘ └─────┘  │   │
│  │  │ GPIO25 ──────┼───┼─► DAC   │                    │   │
│  │  └──────────────┘   │  Out    │  ┌──────────────┐  │   │
│  │                     │         │  │ MPU6050      │  │   │
│  └─────────────────────┘         │  │ (Feedback)   │  │   │
│                                   │  └──────────────┘  │   │
│                                   └─────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

### Flujo de Datos

```
┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
│ Captura │ ──► │  Proc.  │ ──► │  Trans. │ ──► │ Control │
│ IMU 50Hz│     │ Datos   │     │ ESP-NOW │     │ Servos  │
└─────────┘     └─────────┘     └─────────┘     └─────────┘
    │               │                 │               │
    │ Accel X,Y,Z   │ handPosition    │ Broadcast     │ PWM
    │ Gyro  X,Y,Z   │ Timestamp       │ 2.4GHz        │ 0-180°
    └───────────────┴─────────────────┴───────────────┘
                            │
                            ├──► DAC Output (GPIO25)
                            └──► Análisis Osciloscopio
```

---

## 📐 Especificaciones Técnicas

### Hardware - Transmisor (Guante)

| Componente | Modelo | Especificación | Función |
|------------|--------|----------------|---------|
| **Microcontrolador** | ESP32 WROOM | 240MHz, WiFi 2.4GHz | Control y comunicación |
| **IMU** | MPU6050 | ±8G, ±500°/s | Captura de movimiento |
| **DAC** | Integrado | 8-bit, GPIO25/26 | Salida analógica |
| **Comunicación** | ESP-NOW | 2.4GHz, <1ms latencia | Transmisión inalámbrica |
| **Alimentación** | USB/Batería | 3.3V, ~150mA | Fuente de poder |

#### Pinout ESP32 WROOM
```
GPIO 21  ──► I2C SDA (MPU6050)
GPIO 22  ──► I2C SCL (MPU6050)
GPIO 25  ──► DAC1 Output (análisis)
GPIO 2   ──► LED indicador
```

### Hardware - Receptor (Brazo)

| Componente | Modelo | Especificación | Función |
|------------|--------|----------------|---------|
| **Microcontrolador** | ESP32-S3 | 240MHz, WiFi 2.4GHz | Control de actuadores |
| **Servomotores** | MG90S (x2) | 180°, 1.8kg·cm | Actuación del brazo |
| **IMU Feedback** | MPU6050 | ±8G, ±500°/s | Verificación posición (opcional) |
| **Comunicación** | ESP-NOW | 2.4GHz, receptor | Recepción de datos |
| **Alimentación** | Externa 5V/2A | Servos + ESP | Fuente de poder |

#### Pinout ESP32-S3
```
GPIO 6   ──► PWM Servo 1 (Base)
GPIO 7   ──► PWM Servo 2 (Extremo)
GPIO 8   ──► I2C SDA (MPU6050 feedback)
GPIO 10  ──► I2C SCL (MPU6050 feedback)
GPIO 48  ──► LED indicador
```

⚠️ **Nota importante**: GPIO 9 no se usa por problemas conocidos del ESP32-S3.

### Protocolo de Comunicación

#### Estructura del Paquete ESP-NOW
```cpp
struct struct_message {
  float accelX;           // Aceleración X (m/s²)
  float accelY;           // Aceleración Y (m/s²)
  float accelZ;           // Aceleración Z (m/s²)
  float gyroX;            // Giroscopio X (rad/s)
  float gyroY;            // Giroscopio Y (rad/s)
  float gyroZ;            // Giroscopio Z (rad/s)
  unsigned long timestamp; // Marca de tiempo (ms)
  uint8_t handPosition;   // 0=abajo, 1=arriba
};
// Total: 36 bytes
```

#### Parámetros de Comunicación
- **Frecuencia de envío**: 50Hz (20ms entre paquetes)
- **Modo**: Broadcast (FF:FF:FF:FF:FF:FF)
- **Tasa de éxito**: >99% en distancia <10m
- **Latencia promedio**: ~20ms

---

## 🔌 Diagramas de Conexión

### Sistema Completo

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
│   │  GPIO 25 ────►│ DAC Out    │   │
│   │  GPIO 2  ────►│ LED        │   │
│   │  3.3V    ────►│            │   │
│   │  GND     ────►│            │   │
│   └───────┬─────────────────────┘   │
│           │                         │
│   ┌───────┴─────────┐               │
│   │    MPU6050      │               │
│   │  (Obligatorio)  │               │
│   │                 │               │
│   │  VCC ── 3.3V    │               │
│   │  GND ── GND     │               │
│   │  SCL ── GPIO22  │               │
│   │  SDA ── GPIO21  │               │
│   └─────────────────┘               │
└─────────────────────────────────────┘
              │
              │ ESP-NOW
              │ 50Hz, Broadcast
              ▼
┌─────────────────────────────────────┐
│   BRAZO ROBÓTICO (Receptor)         │
│                                     │
│   ┌─────────────────────────────┐   │
│   │    ESP32-S3                 │   │
│   │    (NO tiene DAC)           │   │
│   │                             │   │
│   │  GPIO 8  ◄────┐ I2C        │   │
│   │  GPIO 10 ◄────┘            │   │
│   │  GPIO 6  ────►│ PWM        │   │
│   │  GPIO 7  ────►│ PWM        │   │
│   │  GPIO 48 ────►│ LED        │   │
│   │  5V (ext)────►│            │   │
│   │  GND     ────►│            │   │
│   └─────┬──────┬──────┬─────────┘   │
│         │      │      │             │
│   ┌─────▼──┐ ┌─▼─────┐│            │
│   │ Servo1 │ │ Servo2 ││            │
│   │ (Base) │ │(Extremo)│           │
│   │        │ │        ││            │
│   │ MG90S  │ │ MG90S  ││            │
│   │ Naranja│ │ Naranja││            │
│   │ → GPI06│ │ → GPI07││            │
│   │ Rojo   │ │ Rojo   ││            │
│   │ → 5V   │ │ → 5V   ││            │
│   │ Marrón │ │ Marrón ││            │
│   │ → GND  │ │ → GND  ││            │
│   └────────┘ └────────┘│            │
│                        │             │
│                  ┌─────▼─────────┐   │
│                  │   MPU6050     │   │
│                  │   (Opcional)  │   │
│                  │   Feedback    │   │
│                  │               │   │
│                  │  VCC ── 3.3V  │   │
│                  │  GND ── GND   │   │
│                  │  SCL ── GPI010│   │
│                  │  SDA ── GPIO8 │   │
│                  └───────────────┘   │
│                                     │
│   ⚡ Fuente 5V/2A externa           │
└─────────────────────────────────────┘
```

### Conexión de Servomotores

```
Vista del Conector del Servo MG90S:
┌─────────────────────────┐
│  Cable Naranja (Signal) │──► GPIO (6 o 7)
│  Cable Rojo    (VCC)    │──► +5V (fuente externa)
│  Cable Marrón  (GND)    │──► GND (común con ESP32)
└─────────────────────────┘

⚠️ IMPORTANTE:
- NO alimentar servos desde pin 3.3V del ESP32
- Usar fuente externa 5V mínimo 2A
- Conectar GND común entre fuente, ESP32 y servos
```

### Conexión I2C - MPU6050

```
MPU6050 → ESP32 WROOM (Guante)
┌──────────┬─────────────┐
│ Pin MPU  │ Pin ESP32   │
├──────────┼─────────────┤
│ VCC      │ 3.3V        │
│ GND      │ GND         │
│ SCL      │ GPIO 22     │
│ SDA      │ GPIO 21     │
│ AD0      │ (sin conex) │
│ INT      │ (sin conex) │
└──────────┴─────────────┘

MPU6050 → ESP32-S3 (Brazo)
┌──────────┬─────────────┐
│ Pin MPU  │ Pin ESP32-S3│
├──────────┼─────────────┤
│ VCC      │ 3.3V        │
│ GND      │ GND         │
│ SCL      │ GPIO 10 ⚠️  │
│ SDA      │ GPIO 8      │
│ AD0      │ (sin conex) │
│ INT      │ (sin conex) │
└──────────┴─────────────┘

⚠️ NO usar GPIO 9 en ESP32-S3
```

### Salida DAC para Análisis

```
Osciloscopio / Analizador de Señales
┌──────────────────────────────────┐
│                                  │
│  Canal 1 ◄────────────────────────── GPIO25 (DAC1)
│                                  │   ESP32 WROOM
│  GND     ◄────────────────────────── GND
│                                  │
└──────────────────────────────────┘

Configuración sugerida:
- Escala vertical: 1V/div
- Escala temporal: 20ms/div
- Acoplamiento: DC
- Trigger: Auto o Rising Edge
```

---

## 💻 Implementación

### Estructura del Código

#### Transmisor (Guante)
```cpp
// Inicialización
- Configurar I2C (GPIO21/22)
- Inicializar MPU6050
- Configurar DAC (GPIO25)
- Inicializar WiFi (modo STA)
- Configurar ESP-NOW (broadcast)

// Loop principal (50Hz)
while(true) {
  - Leer MPU6050 (accel + gyro)
  - Determinar handPosition (Z > 8 o Z < 2)
  - Llenar estructura de datos
  - Enviar por ESP-NOW
  - Generar señal DAC (accelX escalada)
  - Debug cada 25 envíos
}
```

#### Receptor (Brazo)
```cpp
// Inicialización
- Configurar WiFi (modo STA)
- Test de servos (45° → 135° → 90°)
- Configurar ESP-NOW
- Inicializar MPU6050 local (opcional)
- Posicionar servos en centro (90°)

// Callback ESP-NOW
OnDataRecv() {
  - Recibir datos
  - Mapear accelX → ángulo (0-180°)
  - Suavizado incremental (paso variable)
  - Aplicar a servo activo (según handPosition)
  - Debug cada 20 recepciones
}

// Loop principal
while(true) {
  - Timeout → volver a posición segura (90°)
  - Apagar LED
  - Mostrar contador si no hay datos
}
```

### Algoritmo de Control

#### Mapeo de Aceleración a Ángulo
```cpp
// Entrada: accelX [-10, +10] m/s²
// Salida: ángulo [0, 180] grados

int targetAngle = map((int)(accelX * 10), -100, 100, 0, 180);
targetAngle = constrain(targetAngle, 0, 180);

// Relación:
// accelX = -10 m/s² → 0°   (máximo izquierda)
// accelX = 0 m/s²   → 90°  (centro)
// accelX = +10 m/s² → 180° (máximo derecha)
```

#### Suavizado Incremental
```cpp
// Movimiento gradual hacia objetivo (reduce temblor)
int currentPos = servoPosition;
int diff = targetAngle - currentPos;

// Paso adaptativo: rápido si lejos, lento si cerca
int step = (abs(diff) > 10) ? 5 : 1;

if (diff > 0) {
  newPos = min(currentPos + step, targetAngle);
} else if (diff < 0) {
  newPos = max(currentPos - step, targetAngle);
}

servo.write(newPos);
```

#### Lógica de Selección de Servo
```cpp
// Basada en orientación vertical de la mano

if (accelZ > 8.0) {
  activeServo = 1;  // Mano ARRIBA → Servo2 (Extremo)
} else if (accelZ < 2.0) {
  activeServo = 0;  // Mano ABAJO → Servo1 (Base)
}
// Si Z entre 2-8: mantiene último estado (histéresis)
```

### Configuraciones Críticas

#### ESP32-S3 (Receptor)
```
Arduino IDE:
- Board: "ESP32S3 Dev Module"
- USB CDC On Boot: Enabled  ← CRÍTICO
- USB Mode: Hardware CDC and JTAG
- Flash Size: 8MB
- Partition Scheme: Default
- Upload Speed: 921600
```

#### ESP32 WROOM (Transmisor)
```
Arduino IDE:
- Board: "ESP32 Dev Module"
- Flash Size: 4MB
- Partition Scheme: Default
- Upload Speed: 921600
```

---

## 📊 Resultados y Validación

### Métricas de Performance

| Métrica | Valor Obtenido | Objetivo | Estado |
|---------|----------------|----------|--------|
| **Latencia comunicación** | ~20ms | <50ms | ✅ Cumplido |
| **Frecuencia de muestreo** | 50Hz | 50Hz | ✅ Cumplido |
| **Tasa de éxito ESP-NOW** | >99% | >95% | ✅ Cumplido |
| **Rango de operación** | ~10m | >5m | ✅ Cumplido |
| **Estabilidad servo** | ±5° | N/A* | ✅ Baseline |
| **Consumo transmisor** | ~150mA | <200mA | ✅ Cumplido |
| **Consumo receptor** | ~1.2A | <2A | ✅ Cumplido |

*Sin filtros, el temblor de ±5° es esperado y sirve como baseline para siguientes sprints.

### Análisis de Señal DAC

**Observaciones en osciloscopio (GPIO25)**:
- Señal de 0-3.3V proporcional a accelX
- Ruido visible: ±100mV (esperado sin filtros)
- Frecuencia de actualización: 50Hz
- Forma de onda: Escalones con ruido superpuesto

**Espectro de frecuencias**:
- Componente principal: 0-5Hz (movimiento de la mano)
- Ruido de alta frecuencia: >10Hz
- Pico notable: 50Hz (frecuencia de muestreo)

### Pruebas Realizadas

#### Test 1: Estabilidad en Reposo
**Procedimiento**: Mano quieta en posición horizontal  
**Resultado**: Servo oscila ±5° alrededor de 90°  
**Conclusión**: Temblor moderado esperado sin filtros ✅

#### Test 2: Respuesta a Movimiento
**Procedimiento**: Inclinar mano izquierda-derecha  
**Resultado**: Servo sigue movimiento con pequeño lag  
**Conclusión**: Control proporcional funcional ✅

#### Test 3: Cambio de Servo
**Procedimiento**: Voltear mano (arriba/abajo)  
**Resultado**: Cambia entre Servo1 y Servo2 correctamente  
**Conclusión**: Lógica de selección funcional ✅

#### Test 4: Timeout y Seguridad
**Procedimiento**: Apagar transmisor  
**Resultado**: Servos vuelven gradualmente a 90° (posición segura)  
**Conclusión**: Sistema seguro ante pérdida de comunicación ✅

#### Test 5: Rango de Operación
**Procedimiento**: Aumentar distancia entre dispositivos  
**Resultado**: Funcional hasta ~12m en línea de vista  
**Conclusión**: Rango adecuado para aplicación ✅

### Problemas Encontrados y Soluciones

#### Problema 1: ESP32-S3 no mostraba Serial Monitor
**Causa**: Configuración "USB CDC On Boot" deshabilitada  
**Solución**: Habilitar en Arduino IDE  
**Resultado**: ✅ Resuelto

#### Problema 2: MAC Address mostraba 00:00:00:00:00:00
**Causa**: No había delay después de WiFi.mode()  
**Solución**: Agregar delay(100) después de WiFi.mode(WIFI_STA)  
**Resultado**: ✅ Resuelto

#### Problema 3: Error "Peer interface is invalid"
**Causa**: No se especificaba interfaz WiFi en peer  
**Solución**: Agregar `peerInfo.ifidx = WIFI_IF_STA`  
**Resultado**: ✅ Resuelto

#### Problema 4: Servos temblaban excesivamente
**Causa**: Datos sin filtrar + mapeo directo  
**Solución**: Implementar suavizado incremental adaptativo  
**Resultado**: ✅ Temblor reducido a nivel aceptable

#### Problema 5: MPU6050 del brazo no detectado
**Causa**: Sensor clone con WHO_AM_I = 0x70 (MPU6500)  
**Solución**: Sistema funciona sin él (feedback opcional)  
**Resultado**: ✅ No crítico, sistema operativo

---

## 🎓 Conclusiones

### Logros Técnicos

1. **Comunicación ESP-NOW Estable**
   - Latencia <20ms consistente
   - Tasa de éxito >99%
   - Rango operativo adecuado (>10m)

2. **Control Proporcional Funcional**
   - Mapeo intuitivo aceleración → ángulo
   - Suavizado incremental reduce temblor
   - Lógica de selección de servo robusta

3. **Arquitectura Escalable**
   - Código modular y bien estructurado
   - Fácil agregar filtros (Sprints 2 y 3)
   - Feedback MPU opcional implementado

4. **Sistema Seguro**
   - Timeout con retorno a posición segura
   - Test de servos al inicio
   - Indicadores LED de estado

### Baseline Establecida

**Sin filtros, el sistema presenta**:
- Temblor de ±5° en reposo (ruido del MPU6050)
- Latencia de ~20ms (comunicación + procesamiento)
- Movimiento funcional pero no suave

**Esta baseline es crítica para**:
- Comparar efectividad de filtros (Sprint 2 y 3)
- Cuantificar mejoras en estabilidad
- Validar reducción de ruido

### Aprendizajes Clave

1. **ESP32-S3 vs ESP32 WROOM**
   - S3 NO tiene DAC → WROOM en guante
   - S3 requiere USB CDC On Boot habilitado
   - GPIO 9 del S3 problemático → usar GPIO 10

2. **ESP-NOW vs WiFi**
   - Latencia mucho menor que WiFi tradicional
   - No requiere router (peer-to-peer)
   - Broadcast simplifica arquitectura

3. **Control de Servos**
   - Suavizado incremental esencial
   - Paso adaptativo mejora performance
   - Timeout de seguridad crítico

4. **Sensores IMU**
   - MPU6050 tiene clones con IDs diferentes
   - Ruido significativo sin filtros
   - Feedback del brazo útil pero no crítico

### Próximos Pasos (Sprint 2 y 3)

**Sprint 2: Filtro de Media Móvil**
- Implementar filtro en transmisor
- Reducir temblor ~60%
- Evaluar trade-off suavidad/latencia

**Sprint 3: Filtro de Kalman**
- Filtrado óptimo (estimación de estado)
- Fusión de sensores (guante + brazo)
- Reducir temblor ~90%

### Aplicaciones Futuras

- Teleoperación industrial
- Rehabilitación médica
- Educación en robótica
- Control de drones/robots
- Interfaces humano-máquina

---

## 📚 Referencias Técnicas

### Datasheets
- ESP32 WROOM: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf
- ESP32-S3: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
- MPU6050: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
- MG90S Servo: Especificaciones del fabricante

### Librerías Utilizadas
- ESP32 Arduino Core v3.x
- Adafruit_MPU6050 v2.2.4
- Adafruit_Sensor v1.1.9
- ESP32Servo v3.0.0

### Herramientas
- Arduino IDE 2.3.2
- I2C Scanner (herramienta de diagnóstico)
- Obtener_MAC (herramienta de configuración)

---

## 📁 Estructura del Repositorio

```
Sprint1_SinFiltro/
├── Transmisor_Guante/
│   └── Transmisor_Guante.ino       # Código del guante (ESP32 WROOM)
│
├── Receptor_Brazo/
│   └── Receptor_Brazo.ino          # Código del brazo (ESP32-S3)
│
├── I2C_Scanner_Auto/
│   └── I2C_Scanner_Auto.ino        # Herramienta diagnóstico I2C
│
├── Obtener_MAC_WROOM/
│   └── Obtener_MAC_WROOM.ino       # Herramienta obtener MAC (WROOM)
│
├── Obtener_MAC_S3/
│   └── Obtener_MAC_S3.ino          # Herramienta obtener MAC (S3)
│
└── Sprint1.md                       # Este documento
```

---

## 👥 Equipo y Créditos

**Desarrollo**: Laboratorio de Señales  
**Institución**: Universidad Militar Nueva Granada  
**Fecha**: Noviembre 2025  
**Sprint**: 1 de 3

---

## 📝 Notas de Versión

**v1.0 - Sprint 1 Completado**
- ✅ Sistema funcional sin filtros
- ✅ Comunicación ESP-NOW estable
- ✅ Control de 2 servos implementado
- ✅ Salida DAC para análisis
- ✅ Baseline establecida
- ✅ Documentación completa

**Próxima versión**: Sprint 2 - Filtro de Media Móvil

---

*Documento generado: Noviembre 2025*  
*Sprint 1: Sistema de Teleoperación de Brazo Robótico 2DOF*
