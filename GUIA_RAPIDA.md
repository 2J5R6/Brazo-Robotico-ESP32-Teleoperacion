# 🚀 GUÍA RÁPIDA - CAMBIO ENTRE SPRINTS

## Autor
**Julián Andrés Rosas Sánchez**  
Ingeniería Mecatrónica  
Universidad Militar Nueva Granada

---

## ✅ Sprint 1 - COMPLETADO

Ya probaste y funciona. Los servos se mueven con temblor (esperado).

---

## 🔄 CÓMO PASAR A SPRINT 2

### Paso 1: Cargar Transmisor Sprint 2
1. Abrir: `Sprint2_FiltroMovil/Transmisor_Guante/Transmisor_Guante.ino`
2. Board: **ESP32 Dev Module**
3. Upload al ESP32 WROOM (guante)

**Deberías ver**:
```
=========================================
   SPRINT 2 - TRANSMISOR (Guante)
   CON FILTRO DE MEDIA MÓVIL
=========================================

✓ DAC configurado
✓ I2C GPIO21/22
✓ MPU6050 OK
✓ Calibrando Filtro de Kalman... OK (50 muestras)
✓ MAC: 20:E7:C8:67:4D:E4
✓ ESP-NOW inicializado
✓ Peer broadcast agregado

=========================================
   SISTEMA LISTO - FILTRO ACTIVO
=========================================
Frecuencia: 50Hz | Filtro: Media móvil (10 muestras)

AccelX(RAW):2.34 → FILT:2.10 | ...
```

**Nota importante**: Ahora muestra **RAW** y **FILT** para que veas la diferencia.

### Paso 2: Cargar Receptor Sprint 2
1. Abrir: `Sprint2_FiltroMovil/Receptor_Brazo/Receptor_Brazo.ino`
2. Board: **ESP32S3 Dev Module**
3. **USB CDC On Boot: Enabled** ← IMPORTANTE
4. Upload al ESP32-S3 (brazo)

**Deberías ver**:
```
========================================
   SPRINT 2 - RECEPTOR
   CON FILTRO DE MEDIA MÓVIL
========================================

[1/5] WiFi... OK
[2/5] Servos... OK (GPIO6, GPIO7)
    Testeando servos... OK (movieron)
[3/5] ESP-NOW... OK
[4/5] MPU6500 Brazo (feedback)... OK/NO DETECTADO
[5/5] Inicialización completa

========================================
   SISTEMA LISTO - FILTRADO ACTIVO
========================================
Datos recibidos YA están filtrados

✓ RX FILTRADO | AccelX:2.10 | Target:104° | ...
```

### Paso 3: Probar
- Mueve la mano
- **Deberías notar**: Menos temblor que Sprint 1
- **Puede haber**: Pequeño lag (normal con filtro)

---

## 🎯 CÓMO PASAR A SPRINT 3

### Paso 1: Cargar Transmisor Sprint 3
1. Abrir: `Sprint3_FiltroKalman/Transmisor_Guante/Transmisor_Guante.ino`
2. Board: **ESP32 Dev Module**
3. Upload al ESP32 WROOM

**Deberías ver**:
```
=========================================
   SPRINT 3 - TRANSMISOR (Guante)
   CON FILTRO DE KALMAN
=========================================

✓ MPU6050 OK
✓ Calibrando Filtro de Kalman... OK (50 muestras)

=========================================
   SISTEMA LISTO - KALMAN ACTIVO
=========================================
Frecuencia: 50Hz | Filtro: Kalman (Q=0.01, R=0.1)

AccelX(RAW):2.34 → KALMAN:2.15 | ...
```

### Paso 2: Cargar Receptor Sprint 3
1. Abrir: `Sprint3_FiltroKalman/Receptor_Brazo/Receptor_Brazo.ino`
2. Board: **ESP32S3 Dev Module**
3. **USB CDC On Boot: Enabled**
4. Upload al ESP32-S3

**Deberías ver**:
```
========================================
   SPRINT 3 - RECEPTOR
   CON FILTRO DE KALMAN + FEEDBACK
========================================

[1/5] WiFi... OK
[2/5] Servos... OK
    Testeando servos... OK
[3/5] ESP-NOW... OK
[4/5] MPU6500 Brazo (FEEDBACK)... OK ✓ (fusión activa)
    Calibrando Kalman local... OK
[5/5] Inicialización completa

========================================
   SISTEMA LISTO - KALMAN + FEEDBACK
========================================
Filtro Kalman: GUANTE + BRAZO
MPU local: ACTIVO (fusión)
Precisión: MÁXIMA

✓ RX KALMAN | Guante:2.15 | Target:104° | Brazo:2.18 | Correg:103° | Real:103° | S2
```

### Paso 3: Observar Fusión de Sensores

Cada 3 segundos verás:
```
╔════════════════════════════════════╗
║  REPORTE FUSIÓN DE SENSORES       ║
╠════════════════════════════════════╣
║ Guante AccelX: 2.15 m/s²          ║
║ Brazo  AccelX: 2.18 m/s²          ║
║ Servo1: 90° | Servo2: 103°        ║
║ Activo: Servo2                    ║
╚════════════════════════════════════╝
```

**Esto es ÚNICO de Sprint 3**: Comparación en tiempo real guante vs brazo.

---

## 🔬 COMPARACIÓN VISUAL

### Sprint 1 (Sin Filtro)
```
Accel X:2.3 Y:1.5 Z:9.8 | Mano:↑ | ✓
Accel X:2.7 Y:1.3 Z:9.6 | Mano:↑ | ✓  ← Salta mucho
Accel X:1.9 Y:1.6 Z:10.1 | Mano:↑ | ✓
```
**Servo**: Tiembla, movimiento brusco

### Sprint 2 (Media Móvil)
```
AccelX(RAW):2.34 → FILT:2.10 | ...
AccelX(RAW):2.67 → FILT:2.15 | ...  ← Filtrado suaviza
AccelX(RAW):1.89 → FILT:2.12 | ...
```
**Servo**: Más estable, pequeño lag

### Sprint 3 (Kalman)
```
AccelX(RAW):2.34 → KALMAN:2.15 | ...
AccelX(RAW):2.67 → KALMAN:2.22 | ...  ← Sigue rápido y suave
AccelX(RAW):1.89 → KALMAN:2.18 | ...
```
**Servo**: Muy estable, mínimo lag, autocorrección

---

## 📊 EXPERIMENTO SUGERIDO

### Test Completo de los 3 Sprints

1. **Grabar video** de los servos en cada sprint
2. **Mover mano** con el mismo patrón
3. **Comparar**:
   - Estabilidad en reposo
   - Suavidad en movimiento
   - Velocidad de respuesta

### Análisis de Señal DAC

Con osciloscopio en GPIO25:

**Sprint 1**: Señal ruidosa, picos aleatorios
**Sprint 2**: Señal suavizada, curvas más redondeadas
**Sprint 3**: Señal limpia, sigue tendencias sin ruido

---

## 🛠️ AJUSTE DE PARÁMETROS

### Sprint 2: FILTER_SIZE

En `Transmisor_Guante.ino` línea ~18:
```cpp
#define FILTER_SIZE 10  // Cambiar este valor
```

**Probar**:
- `FILTER_SIZE 5`: Menos suave, más rápido
- `FILTER_SIZE 10`: Balance (default)
- `FILTER_SIZE 20`: Muy suave, más lag

### Sprint 3: Q y R

En `Transmisor_Guante.ino` línea ~112:
```cpp
KalmanFilter kalmanX(0.01, 0.1, 1.0, 0.0);
//                    Q     R
```

**Probar**:
- Q más alto (0.05): Más reactivo, menos suavizado
- Q más bajo (0.005): Más suave, más lento
- R más alto (0.5): Confía menos en sensor
- R más bajo (0.05): Confía más en sensor

**Recomendación**: Empezar con valores default, ajustar si necesario.

---

## ❓ TROUBLESHOOTING

### Sprint 2: "No reduce el temblor"
- Aumentar `FILTER_SIZE` a 15-20
- Verificar que debug muestre RAW ≠ FILT

### Sprint 3: "No veo fusión"
- Verificar que MPU del brazo esté conectado
- Debe decir "OK ✓ (fusión activa)"
- Debe mostrar "Brazo AccelX" en reportes

### Sprint 3: "Movimiento extraño"
- Ajustar corrección en línea ~96 del Receptor:
```cpp
correctedAngle = targetAngle + (error / 4);
//                                      ↑ Cambiar a 5-10
```

---

## 🎓 CONCLUSIONES ESPERADAS

### Sprint 1
- Ruido del MPU6050 es significativo
- Control básico funciona
- Necesario filtrar

### Sprint 2
- Media móvil reduce temblor efectivamente
- Introduce lag perceptible
- Simple pero efectivo

### Sprint 3
- Kalman superior a media móvil
- Fusión MPU mejora precisión
- Óptimo para aplicación real

---

## 📝 CHECKLIST FINAL

### Sprint 2
- [ ] Transmisor muestra RAW vs FILT
- [ ] Receptor dice "FILTRADO ACTIVO"
- [ ] Temblor visiblemente menor que Sprint 1
- [ ] Debug cada 20 recepciones funciona

### Sprint 3
- [ ] Transmisor calibra Kalman (50 muestras)
- [ ] Receptor dice "FUSIÓN ACTIVA"
- [ ] Muestra reporte cada 3 segundos
- [ ] Comparación Guante vs Brazo visible
- [ ] Temblor mínimo
- [ ] Respuesta rápida

---

¡TODO LISTO! Ahora tienes los 3 sprints completos y documentados. 🎉

**Siguiente paso**: Probar Sprint 2 y comparar con Sprint 1.
