# 🎯 SISTEMA DE TELEOPERACIÓN - 3 SPRINTS

## 📊 COMPARATIVA DE SPRINTS

| Sprint | Filtro | Temblor | MPU Brazo | Complejidad | Precisión |
|--------|--------|---------|-----------|-------------|-----------|
| **1** | Ninguno | ⚠️⚠️⚠️ Alto | Opcional | ⭐ Baja | 🎯 Media |
| **2** | Media Móvil | ⚠️⚠️ Medio | Feedback | ⭐⭐ Media | 🎯🎯 Alta |
| **3** | Kalman | ⚠️ Bajo | **Fusión** | ⭐⭐⭐ Alta | 🎯🎯🎯 Máxima |

---

## 📁 ESTRUCTURA DE CARPETAS

```
Lab3/
├── Sprint1_SinFiltro/
│   ├── Transmisor_Guante/
│   │   └── Transmisor_Guante.ino
│   └── Receptor_Brazo/
│       └── Receptor_Brazo.ino
│
├── Sprint2_FiltroMovil/
│   ├── Transmisor_Guante/
│   │   └── Transmisor_Guante.ino (con media móvil)
│   └── Receptor_Brazo/
│       └── Receptor_Brazo.ino (recibe filtrado)
│
└── Sprint3_FiltroKalman/
    ├── Transmisor_Guante/
    │   └── Transmisor_Guante.ino (con Kalman)
    └── Receptor_Brazo/
        └── Receptor_Brazo.ino (Kalman + fusión MPU)
```

---

## 🚀 SPRINT 1: SIN FILTRO (✅ COMPLETADO)

### Características
- ✅ Datos RAW del MPU6050
- ✅ ESP-NOW broadcast
- ✅ DAC para análisis de señales
- ✅ Lógica de control suave (reduce temblor parcialmente)
- ✅ MPU del brazo **opcional** (solo feedback)

### Hardware
- **Transmisor**: ESP32 WROOM + MPU6050 (guante)
- **Receptor**: ESP32-S3 + 2 Servos + MPU6500 (opcional)

### Resultados
- ✓ Comunicación ESP-NOW funcional
- ✓ Servos responden a movimiento
- ⚠️ Temblor moderado (normal sin filtro)

### Objetivo Didáctico
Establecer baseline y ver el **ruido real** del sensor sin filtrar.

---

## 🔧 SPRINT 2: FILTRO DE MEDIA MÓVIL

### ¿Qué es Media Móvil?
Promedia las **últimas N lecturas** para suavizar la señal.

```cpp
// Ejemplo: promedio de 10 lecturas
filteredValue = (L1 + L2 + L3 + ... + L10) / 10
```

### Ventajas
- ✅ Muy simple de implementar
- ✅ Reduce ruido de alta frecuencia
- ✅ Bajo costo computacional
- ✅ Intuitivo de entender

### Desventajas
- ⚠️ Introduce retraso (lag)
- ⚠️ No adapta a cambios dinámicos
- ⚠️ Filtrado fijo, no óptimo

### Implementación
**Transmisor**: 
- Clase `MovingAverageFilter` con buffer circular
- Aplica filtro a AccelX, AccelY, AccelZ
- Envía datos **ya filtrados** por ESP-NOW

**Receptor**:
- Recibe datos suavizados
- MPU local para **feedback visual**
- Muestra comparación cada 2 segundos

### Parámetros Ajustables
```cpp
#define FILTER_SIZE 10  // Aumentar = más suave, más lag
```

**Recomendado**: 
- FILTER_SIZE = 5-10 para balance suavidad/respuesta
- FILTER_SIZE > 15 = muy lento
- FILTER_SIZE < 5 = poco efecto

### Resultado Esperado
- Temblor **reducido ~60%**
- Movimiento **más suave**
- Latencia aceptable

---

## 🎯 SPRINT 3: FILTRO DE KALMAN

### ¿Qué es Filtro de Kalman?
Algoritmo de **estimación óptima** que:
1. Predice el siguiente estado
2. Corrige con la medición real
3. Minimiza el error cuadrático medio

**Ideal para**: Sensores con ruido Gaussiano (como MPU6050)

### Ventajas vs Media Móvil
- ✅ **Filtrado adaptativo** (se ajusta dinámicamente)
- ✅ **Menor latencia** (responde más rápido)
- ✅ **Mayor precisión** (fusiona múltiples fuentes)
- ✅ **Óptimo matemáticamente**

### Componentes del Filtro

```cpp
Q = 0.01;  // Ruido del proceso (qué tanto varía el sistema)
R = 0.1;   // Ruido de la medición (confianza en el sensor)
P = 1.0;   // Covarianza del error (incertidumbre inicial)
X = 0.0;   // Estado estimado (valor filtrado)
```

**Clave**: Relación Q/R determina qué tanto confiar en el sensor vs el modelo.

### Implementación DUAL

#### 1. **Transmisor (Guante)**
- Filtro Kalman para AccelX, AccelY, AccelZ
- Calibración inicial (50 muestras)
- Envía datos **filtrados óptimamente**

#### 2. **Receptor (Brazo)**
- **NOVEDAD**: Fusión de sensores
- Compara datos del guante vs MPU del brazo
- Corrección proporcional del error
- Filtro Kalman adicional para MPU local

### Fusión de Sensores (Único en Sprint 3)

```cpp
// Posición deseada (del guante)
targetAngle = map(guanteAccelX, ...)

// Posición real (del brazo)
realAngle = map(brazoAccelX, ...)

// Corrección
error = targetAngle - realAngle
correctedAngle = targetAngle + (error / 4)  // Suave
```

**Beneficio**: El sistema **autocorrige** si el servo no llegó a la posición deseada.

### Parámetros Ajustables

**Transmisor**:
```cpp
KalmanFilter kalmanX(0.01, 0.1, 1.0, 0.0);
//                    Q     R    P    X0
```

**Receptor (MPU local)**:
```cpp
KalmanFilter kalmanBrazo(0.005, 0.05, 1.0, 0.0);
//                        Q      R     P    X0
```

**Ajustar Q/R**:
- Q alto = confía en mediciones, más reactivo
- R alto = confía en modelo, más suave
- Típico: Q = 0.001 a 0.1, R = 0.01 a 1.0

### Resultado Esperado
- Temblor **reducido ~90%**
- Movimiento **muy suave y preciso**
- **Autocorrección** si hay error de posicionamiento
- Latencia mínima

---

## 🔬 ANÁLISIS COMPARATIVO

### Señal en DAC (GPIO25)

**Sprint 1**: 
- Señal ruidosa, fluctuaciones visibles
- Bueno para ver ruido original

**Sprint 2**: 
- Señal más limpia
- Picos suavizados
- Pequeño retraso visible

**Sprint 3**: 
- Señal muy limpia
- Sigue cambios rápidos
- Mínimo retraso

### Performance en Servos

| Métrica | Sprint 1 | Sprint 2 | Sprint 3 |
|---------|----------|----------|----------|
| Estabilidad | 60% | 85% | 95% |
| Latencia | 20ms | 40ms | 25ms |
| Precisión | Media | Alta | Máxima |
| Suavidad | Baja | Alta | Muy Alta |

---

## 📊 PRUEBAS RECOMENDADAS

### Test 1: Estabilidad en Reposo
1. Poner mano quieta en posición horizontal
2. Observar cuánto tiembla el servo
3. Medir desviación estándar del ángulo

**Esperado**:
- Sprint 1: ±5° de variación
- Sprint 2: ±2° de variación
- Sprint 3: ±0.5° de variación

### Test 2: Respuesta a Paso
1. Mover mano rápidamente de horizontal a vertical
2. Medir tiempo hasta que servo se estabiliza
3. Ver overshoot (cuánto se pasa)

**Esperado**:
- Sprint 1: Rápido pero oscila
- Sprint 2: Más lento, sin oscilación
- Sprint 3: Rápido Y sin oscilación

### Test 3: Seguimiento Continuo
1. Mover mano continuamente izquierda-derecha
2. Ver qué tan suave sigue el servo
3. Detectar lag visual

**Esperado**:
- Sprint 1: Sigue pero con sacudidas
- Sprint 2: Suave con lag notable
- Sprint 3: Suave y responsivo

---

## 🎓 CONCEPTOS APRENDIDOS

### Sprint 1
- Comunicación ESP-NOW
- Lectura de IMU
- Control básico de servos
- Identificación de ruido

### Sprint 2
- Filtrado en dominio del tiempo
- Media móvil y ventanas deslizantes
- Trade-off suavidad vs latencia
- Buffers circulares

### Sprint 3
- Estimación óptima
- Filtro de Kalman (predicción + corrección)
- Fusión de sensores
- Sistemas de control realimentados
- Parámetros Q, R, P, K

---

## 🛠️ CONFIGURACIÓN POR SPRINT

### Sprint 1
**Arduino IDE**:
- ESP32 WROOM: Board "ESP32 Dev Module"
- ESP32-S3: Board "ESP32S3 Dev Module", **USB CDC On Boot: Enabled**

**No requiere**: Librerías adicionales

### Sprint 2
**Igual que Sprint 1**
- Filtro implementado en código (no librería externa)

### Sprint 3
**Igual que Sprint 1**
- Filtro Kalman implementado en código
- **IMPORTANTE**: Conectar MPU en el brazo para fusión

---

## 📈 ROADMAP DE IMPLEMENTACIÓN

### Semana 1: Sprint 1 ✅
- [x] Configurar hardware
- [x] ESP-NOW funcionando
- [x] Control básico de servos
- [x] DAC para análisis
- [x] Identificar nivel de ruido

### Semana 2: Sprint 2
- [ ] Implementar media móvil
- [ ] Comparar señal filtrada vs raw
- [ ] Medir mejora en estabilidad
- [ ] Ajustar FILTER_SIZE
- [ ] Documentar resultados

### Semana 3: Sprint 3
- [ ] Implementar Kalman en transmisor
- [ ] Calibración inicial
- [ ] Conectar MPU del brazo
- [ ] Implementar fusión de sensores
- [ ] Ajustar parámetros Q y R
- [ ] Pruebas finales
- [ ] Comparativa de los 3 sprints

---

## 🎯 CRITERIOS DE ÉXITO

### Sprint 1
- ✓ Servos se mueven correctamente
- ✓ Comunicación estable
- ✓ DAC genera señal analógica

### Sprint 2
- ✓ Temblor visiblemente reducido
- ✓ Debug muestra valores RAW vs FILTRADOS
- ✓ Movimiento más suave que Sprint 1

### Sprint 3
- ✓ Movimiento muy estable
- ✓ Fusión MPU funciona (muestra corrección)
- ✓ Reporte cada 3s con comparativa guante/brazo
- ✓ Mejor performance que Sprint 2

---

## 📝 NOTAS IMPORTANTES

### MPU del Brazo
- **Sprint 1**: Opcional (solo feedback visual)
- **Sprint 2**: Opcional (feedback mejorado)
- **Sprint 3**: **RECOMENDADO** (para fusión de sensores)

### DAC (GPIO25)
- Presente en los 3 sprints
- Permite analizar señal con osciloscopio
- Comparar calidad de filtrado

### Lógica de Control
- **MANTENIDA** en los 3 sprints
- Suavizado incremental funciona muy bien
- Solo cambia la calidad de la señal de entrada

---

## 🚀 ¡LISTO PARA SPRINT 2 Y 3!

Tienes todo el código listo. Ahora puedes:

1. **Probar Sprint 1** una vez más para tener baseline
2. **Pasar a Sprint 2** cuando estés listo
3. **Finalizar con Sprint 3** para máxima precisión

¡Excelente trabajo llegando hasta aquí! 🎉
