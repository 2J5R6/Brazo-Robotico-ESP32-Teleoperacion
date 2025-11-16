# SPRINT 1 - Sistema de Tele-operación Sin Filtros

## 📋 Descripción General

Este es el primer sprint del proyecto de tele-operación del brazo robótico 2DOF. En esta fase se implementa el sistema básico **sin filtros avanzados** para:

1. ✅ Capturar movimiento de la mano con IMU MPU6050
2. ✅ Transmitir datos por ESP-NOW en modo broadcast
3. ✅ Generar señal DAC para análisis de ruido
4. ✅ Controlar 2 servomotores según posición de la mano
5. ✅ Implementar selección de servo por eje Z (arriba/abajo)

---

## 🔧 Hardware Requerido

### Transmisor (Guante)
- **1x ESP32 WROOM** (tiene DAC para análisis de señal)
- **1x MPU6050** (sensor inercial 6DOF) - **OBLIGATORIO**
- **Cables y protoboard**
- **Guante o soporte ergonómico** para montar la IMU

### Receptor (Brazo Robótico)
- **1x ESP32-S3** (va SIEMPRE con el brazo robótico)
- **1x MPU6050** (feedback de posición real) - **OPCIONAL**
- **2x Servomotores MG90S**
- **Fuente de alimentación 5V** (mínimo 2A)
- **Estructura mecánica del brazo** (de la práctica anterior)

**NOTAS IMPORTANTES**: 
- ✅ ESP32 WROOM tiene DAC (GPIO25/26) - Por eso va en el guante
- ❌ ESP32-S3 NO tiene DAC - Por eso va en el brazo
- ✅ Sistema usa 2x MPU6050: uno para CONTROL (guante) y otro para FEEDBACK (brazo)
- ⚠️ El MPU6050 del guante es CRÍTICO - sin él no funciona nada
- ⚠️ El MPU6050 del brazo es opcional - solo para verificar posición real

---

## 📡 Conexiones

### TRANSMISOR (ESP32 WROOM + MPU6050)

```
MPU6050          ESP32 WROOM
--------         -----------
VCC      ──────► 3.3V
GND      ──────► GND
SDA      ──────► GPIO 21 (I2C SDA estándar)
SCL      ──────► GPIO 22 (I2C SCL estándar)

Salida DAC:      GPIO 25 (DAC1 - para osciloscopio/análisis)
LED:             GPIO 2 (LED integrado)
```

**IMPORTANTE**: ESP32 WROOM tiene DAC, por eso va en el guante

### RECEPTOR (ESP32-S3 + Servos + MPU6050)

```
MPU6050 (Brazo)  ESP32-S3
---------------  --------
VCC      ──────► 3.3V
GND      ──────► GND
SDA      ──────► GPIO 8 (I2C SDA)
SCL      ──────► GPIO 10 (I2C SCL) ⚠️ NO usar GPIO 9

Servo1 (Base)    ESP32-S3
-------------    --------
Señal    ──────► GPIO 12
VCC      ──────► 5V (externa)
GND      ──────► GND común

Servo2 (Extremo) ESP32-S3
---------------  --------
Señal    ──────► GPIO 13
VCC      ──────► 5V (externa)
GND      ──────► GND común

LED:             GPIO 48 (LED integrado ESP32-S3)
```

⚠️ **IMPORTANTE ESP32-S3**: 
- GPIO 9 tiene problemas - usar GPIO 10 para SCL
- NUNCA usar GPIO 19/20 (USB D-/D+)
- Conectar servos a fuente externa de 5V/2A mínimo
- MPU6050 del brazo es opcional - sistema funciona sin él

---

## 📚 Librerías Necesarias

Instalar desde el Gestor de Librerías de Arduino IDE:

### Para el Transmisor:
1. **Adafruit MPU6050** (by Adafruit) - **OBLIGATORIO**
2. **Adafruit Unified Sensor** (by Adafruit) - **OBLIGATORIO**

### Para el Receptor:
1. **ESP32Servo** (by Kevin Harrington) - **OBLIGATORIO**
2. **Adafruit MPU6050** (by Adafruit) - **OPCIONAL** (solo si usas MPU en brazo)
3. **Adafruit Unified Sensor** (by Adafruit) - **OPCIONAL** (solo si usas MPU en brazo)

Las librerías de **WiFi** y **ESP-NOW** vienen incluidas con el core de ESP32.

---

## 🚀 Instrucciones de Uso

### 1. Configurar Arduino IDE

```
Herramientas > Placa:
  - Transmisor (Guante): "ESP32 Dev Module"
  - Receptor (Brazo): "ESP32S3 Dev Module"

Herramientas > USB CDC On Boot: "Enabled" (solo para ESP32-S3)
Herramientas > Upload Speed: "921600"
```

### 2. Subir Código al Transmisor (Guante)

1. Abrir `Transmisor_Guante/Transmisor_Guante.ino`
2. Seleccionar **"ESP32 Dev Module"**
3. Seleccionar puerto COM correcto
4. Subir código
5. Abrir Monitor Serial (115200 baud)
6. **Anotar la dirección MAC** que aparece

### 3. Subir Código al Receptor (Brazo)

1. Abrir `Receptor_Brazo/Receptor_Brazo.ino`
2. Seleccionar **"ESP32S3 Dev Module"**
3. Seleccionar puerto COM correcto
4. Subir código
5. Abrir Monitor Serial (115200 baud)

### 4. Prueba del Sistema

1. **Calibración inicial**: Ambos servos deben estar en 90° (posición central)
2. **Mover la mano**: Observar el movimiento de los servos
3. **Cambio de servo**:
   - **Mano ABAJO**: Se activa Servo1 (base)
   - **Mano ARRIBA**: Se activa Servo2 (extremo)

---

## 🎯 Lógica de Control

### Selección de Servo por Eje Z

```
Si aceleración Z > 8.0 m/s²:
  ├─> Mano está ARRIBA (apuntando al cielo)
  └─> Activa SERVO2 (extremo del brazo)

Si aceleración Z < 2.0 m/s²:
  ├─> Mano está ABAJO
  └─> Activa SERVO1 (base del brazo)

Si Z entre 2.0 y 8.0:
  └─> Mantiene último estado
```

### Mapeo de Movimiento

```
Aceleración X: -4G a +4G
       ↓
Ángulo Servo: 0° a 180°
```

---

## 📊 Análisis de Señales (DAC)

### Conexión del Osciloscopio

```
Osciloscopio        ESP32 WROOM (Guante)
------------        --------------------
Canal 1      ────► GPIO 25 (DAC1)
GND          ────► GND
```

**NOTA**: Solo el ESP32 WROOM tiene DAC. El ESP32-S3 NO tiene DAC.

### Parámetros a Medir

1. **Frecuencia de muestreo**: 50 Hz (20ms)
2. **Rango de voltaje DAC**: 0V a 3.3V
3. **Señal representada**: Aceleración X escalada
4. **Observar**:
   - Ruido de alta frecuencia
   - Deriva del sensor (drift)
   - Respuesta a movimientos bruscos
   - Estabilidad en reposo

---

## 🔍 Experimento - Tabla de Datos

Definir puntos de prueba y llenar la siguiente tabla:

| Punto | Posición Esperada (cm) | Posición Medida (cm) | Posición Estimada (cm) |
|-------|------------------------|----------------------|------------------------|
|       | x = ____ , y = ____    | x = ____ , y = ____  | x = ____ , y = ____    |
| 1     |                        |                      |                        |
| 2     |                        |                      |                        |
| 3     |                        |                      |                        |
| 4     |                        |                      |                        |
| 5     |                        |                      |                        |

### Cálculo de Errores

```
Error Esperado vs Medido = |Posición Esperada - Posición Medida|
Error Medido vs Estimado = |Posición Medida - Posición Estimada|
```

---

## 🐛 Solución de Problemas

### Transmisor no envía datos
- ✅ Verificar conexión I2C del MPU6050
- ✅ Revisar que aparezca "MPU6050 detectado" en serial
- ✅ Probar con otro MPU6050

### Receptor no recibe datos
- ✅ Verificar que ambos ESP estén en el mismo canal WiFi
- ✅ Revisar que ESP-NOW se inicialice correctamente
- ✅ Comprobar alimentación de ambos ESP32

### Servos no se mueven
- ✅ Verificar alimentación externa de 5V/2A
- ✅ Revisar conexión de señal (GPIO 12 y 13)
- ✅ Comprobar que el servo no esté dañado

### Movimiento errático
- ✅ **ESPERADO EN SPRINT 1** (sin filtros)
- ✅ Reducir velocidad de movimiento
- ✅ Verificar ruido en señal DAC con osciloscopio

---

## 📈 Resultados Esperados

En este sprint SIN filtros, es normal observar:

- ❌ Ruido visible en la señal DAC
- ❌ Movimientos bruscos o temblorosos de los servos
- ❌ Deriva lenta del sensor (drift)
- ❌ Sensibilidad a vibraciones
- ✅ Respuesta rápida a movimientos
- ✅ Comunicación estable ESP-NOW

**Nota**: Estos problemas se resolverán en Sprint 2 (filtros) y Sprint 3 (Kalman)

---

## 📝 Preguntas del Experimento

Después de realizar las pruebas, responder:

1. **¿Cuál de los errores es mayor?** (Esperado vs Medido) o (Medido vs Estimado)
2. **En promedio ¿Cuánto es el error entre posición esperada y medida?**
3. **¿Qué características observas en la señal del DAC?**
4. **¿El ruido afecta significativamente el control?**
5. **¿Qué estrategias propones para Sprint 2?**

---

## 🔜 Próximos Pasos (Sprint 2)

- Implementar filtros paso bajo
- Promedios móviles
- Filtro complementario
- Comparar resultados con Sprint 1

---

## 📞 Notas Importantes

### Reglas de GPIO para ESP32-S3 (Brazo Robótico)
- ✅ Usar GPIO 1-10, 12-15, 21, 45-48 para OUTPUT
- ❌ NUNCA usar GPIO 19/20 (USB D-/D+)
- ❌ GPIO 9 tiene problemas - usar GPIO 10 para I2C SCL
- ❌ ESP32-S3 NO tiene DAC (por eso va en el brazo, no en el guante)

### GPIO para ESP32 WROOM (Guante)
- ✅ I2C estándar: SDA=GPIO21, SCL=GPIO22
- ✅ DAC1=GPIO25, DAC2=GPIO26
- ✅ LED integrado: GPIO2

### Seguridad
- ⚠️ Desconectar alimentación antes de cambiar conexiones
- ⚠️ No exceder 3.3V en pines GPIO del ESP32
- ⚠️ Servos requieren fuente externa (no conectar a 3.3V)

---

**Fecha**: Noviembre 2025  
**Laboratorio**: Señales - Semestre VI  
**Sprint**: 1 de 3 (Sin Filtros)
