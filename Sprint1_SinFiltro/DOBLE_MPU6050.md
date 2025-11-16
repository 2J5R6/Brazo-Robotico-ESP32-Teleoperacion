# Sistema con Doble MPU6050

## 🎯 ¿Por qué dos sensores MPU6050?

Este sistema utiliza **DOS sensores MPU6050** con propósitos diferentes:

---

## 📡 MPU6050 #1 - EN EL GUANTE (Transmisor)

**Ubicación**: ESP32 WROOM  
**Función**: **ENTRADA DE CONTROL**

```
┌──────────────────────────┐
│   GUANTE DEL OPERADOR    │
│                          │
│     ┌─────────┐          │
│     │ MPU6050 │          │
│     │  #1     │          │
│     └─────────┘          │
│                          │
│  Lee movimientos de      │
│  la MANO del usuario     │
│                          │
└──────────────────────────┘
```

### Propósito:
- ✅ **Capturar** los movimientos de la mano del operador
- ✅ **Generar comandos** de control
- ✅ **Determinar** qué servo activar (mano arriba/abajo)
- ✅ **Transmitir** datos por ESP-NOW
- ✅ **Salida DAC** para análisis en osciloscopio

### Datos que envía:
```cpp
struct_message {
  float accelX;       // Control del servo activo
  float accelY;       // (opcional para 2D)
  float accelZ;       // Determina servo activo (arriba/abajo)
  float gyroX;        // Velocidad angular
  float gyroY;        // Velocidad angular
  float gyroZ;        // Velocidad angular
  unsigned long timestamp;
  uint8_t handPosition;  // 0=abajo, 1=arriba
}
```

### Configuración:
- **I2C**: GPIO21 (SDA), GPIO22 (SCL)
- **Rango**: ±8G acelerómetro
- **Filtro**: 21Hz interno
- **Frecuencia**: 50Hz

**🚨 CRÍTICO**: Este MPU6050 es OBLIGATORIO. Sin él, el sistema no funciona.

---

## 🤖 MPU6050 #2 - EN EL BRAZO (Receptor)

**Ubicación**: ESP32-S3  
**Función**: **FEEDBACK DE POSICIÓN REAL**

```
┌──────────────────────────┐
│   BRAZO ROBÓTICO         │
│                          │
│        Servo2            │
│          ●               │
│          │               │
│     ┌─────────┐          │
│     │ MPU6050 │          │
│     │  #2     │          │
│     └─────────┘          │
│          │               │
│        Servo1            │
│          ●               │
│                          │
│  Lee posición REAL       │
│  del brazo robótico      │
│                          │
└──────────────────────────┘
```

### Propósito:
- ✅ **Verificar** que el brazo se movió correctamente
- ✅ **Comparar** posición comandada vs posición real
- ✅ **Detectar** errores de seguimiento
- ✅ **Monitoreo** para experimentos de Sprint 2 y 3

### Datos que lee:
```cpp
sensors_event_t accel, gyro, temp;
mpuBrazo.getEvent(&accel, &gyro, &temp);

// Posición REAL del brazo
accel.acceleration.x   // Inclinación eje X
accel.acceleration.y   // Inclinación eje Y
accel.acceleration.z   // Gravedad
```

### Configuración:
- **I2C**: GPIO8 (SDA), GPIO10 (SCL) ⚠️ NO GPIO 9
- **Rango**: ±8G acelerómetro
- **Filtro**: 21Hz interno
- **Frecuencia**: Lectura cada 500ms (solo monitoreo)

**💡 OPCIONAL**: El sistema funciona sin este sensor, pero NO tendrás feedback de posición real.

---

## 🔄 Flujo de Datos Completo

```
┌────────────────────┐
│   OPERADOR mueve   │
│   la MANO          │
└──────┬─────────────┘
       │
       ▼
┌────────────────────┐
│   MPU6050 #1       │
│   (Guante)         │
│   Lee accel/gyro   │
└──────┬─────────────┘
       │
       ▼
┌────────────────────┐
│   ESP32 WROOM      │
│   Procesa datos    │
│   + DAC output     │
└──────┬─────────────┘
       │
       │ ESP-NOW
       │ 50 Hz
       ▼
┌────────────────────┐
│   ESP32-S3         │
│   Recibe comando   │
└──────┬─────────────┘
       │
       ▼
┌────────────────────┐
│   Mueve SERVOS     │
│   Servo1 o Servo2  │
└──────┬─────────────┘
       │
       ▼
┌────────────────────┐
│   BRAZO se mueve   │
└──────┬─────────────┘
       │
       ▼
┌────────────────────┐
│   MPU6050 #2       │
│   (Brazo)          │
│   Lee posición     │
│   REAL alcanzada   │
└────────────────────┘
```

---

## 📊 Comparación en Serial Monitor

```
=== TRANSMISOR (Guante) ===
Enviando | AccelX: 2.34 | AccelY: -1.12 | AccelZ: 9.45
         | Mano: ARRIBA | Servo activo: 2
         | DAC: 185/255

         ⬇ ESP-NOW ⬇

=== RECEPTOR (Brazo) ===
Recibido | AccelX: 2.34 | AccelY: -1.12 | AccelZ: 9.45
         | Mano: ARRIBA | Servo activo: 2
         | Ángulo: 132°

--- FEEDBACK MPU6050 BRAZO ---
Posición Real | Accel X: 2.15 | Y: -1.08 | Z: 9.52 m/s²
Servos | S1: 90° | S2: 132° | Activo: S2 (Extremo)
```

### Análisis:
- **Comando**: AccelX = 2.34 m/s²
- **Ángulo calculado**: 132°
- **Posición real**: AccelX = 2.15 m/s²
- **Error**: 0.19 m/s² (aceptable en Sprint 1 sin filtros)

---

## 🎓 Utilidad para los Sprints

### Sprint 1 - Sin Filtros:
```
MPU #1 (Guante): Captura señal RAW con ruido
                 ↓
                Análisis con DAC + Osciloscopio
                 ↓
MPU #2 (Brazo):  Verifica cuánto ruido llega al brazo
                 DOCUMENTA el error sin filtrado
```

### Sprint 2 - Con Filtro de Kalman:
```
MPU #1 (Guante): Captura señal RAW
                 ↓
                Filtro de Kalman
                 ↓
                Análisis con DAC + Osciloscopio
                 ↓
MPU #2 (Brazo):  Verifica mejora en seguimiento
                 COMPARA error vs Sprint 1
```

### Sprint 3 - Con Filtro Promedio Móvil:
```
MPU #1 (Guante): Captura señal RAW
                 ↓
                Filtro Promedio Móvil
                 ↓
                Análisis con DAC + Osciloscopio
                 ↓
MPU #2 (Brazo):  Verifica características diferentes
                 COMPARA con Kalman y sin filtros
```

---

## 🔧 Problema con MPU6050 Clones

### Situación actual:
```
✅ MPU6050 "A" funciona correctamente
❌ MPU6050 "B" NO es detectado por librería Adafruit

Razones comunes:
1. Chip falso/clon de baja calidad
2. Dirección I2C diferente (verificar pin AD0)
3. Pull-ups internos defectuosos
4. Módulo dañado
```

### Solución:
```
GUANTE (obligatorio):  Usar MPU6050 "A" (el que funciona)
BRAZO (opcional):      Probar MPU6050 "B"
                       Si no funciona → Sistema sigue operando
                       Solo pierdes feedback visual
```

---

## 📈 Datos para Experimento

### Tabla de comparación recomendada:

| Sprint | Sensor Guante | DAC Análisis | Sensor Brazo | Error Promedio |
|--------|---------------|--------------|--------------|----------------|
| 1      | RAW           | ✓            | ✓            | ± X.XX m/s²    |
| 2      | Kalman        | ✓            | ✓            | ± X.XX m/s²    |
| 3      | Prom. Móvil   | ✓            | ✓            | ± X.XX m/s²    |

### Gráficas recomendadas:
```
1. Señal del Guante (MPU #1) vs Tiempo
   - RAW vs Filtrada
   - Mostrar reducción de ruido

2. Posición Brazo (MPU #2) vs Tiempo
   - Respuesta a comandos
   - Tiempo de estabilización

3. Error de Seguimiento vs Sprint
   - Comparar efectividad de filtros
```

---

## 🚨 Recordatorio Importante

```
┌─────────────────────────────────────────┐
│  ⚠️ MPU6050 GUANTE = OBLIGATORIO         │
│     Sin él, el sistema NO funciona       │
│                                          │
│  💡 MPU6050 BRAZO = OPCIONAL              │
│     Ayuda con análisis, pero no crítico  │
└─────────────────────────────────────────┘
```

---

**Sprint 1** - Sistema Sin Filtros  
Noviembre 2025
