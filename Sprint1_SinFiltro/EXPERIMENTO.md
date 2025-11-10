# EXPERIMENTO - SPRINT 1 (Sin Filtros)

## Información del Equipo
- **Fecha**: _______________
- **Integrantes**: 
  - _______________
  - _______________
  - _______________
- **Laboratorio**: Señales - Semestre VI

---

## 1. Definición del Espacio de Trabajo

### 1.1 Espacio del Brazo Robótico

| Parámetro | Valor | Unidad |
|-----------|-------|--------|
| Longitud segmento 1 (L1) | ______ | cm |
| Longitud segmento 2 (L2) | ______ | cm |
| Alcance máximo (L1+L2) | ______ | cm |
| Alcance mínimo | ______ | cm |
| Rango angular Servo1 | 0° - 180° | grados |
| Rango angular Servo2 | 0° - 180° | grados |

### 1.2 Espacio del Operador Humano

| Parámetro | Valor | Unidad |
|-----------|-------|--------|
| Rango movimiento X | ±_____ | cm |
| Rango movimiento Y | ±_____ | cm |
| Área de trabajo | ______ | cm² |

### 1.3 Escalado

```
Factor de escala X = (Alcance Robot X) / (Alcance Operador X) = _______
Factor de escala Y = (Alcance Robot Y) / (Alcance Operador Y) = _______
```

---

## 2. Configuración del Sistema

### 2.1 Parámetros de Muestreo

| Parámetro | Valor Configurado | Justificación |
|-----------|-------------------|---------------|
| Frecuencia de muestreo | 50 Hz | ___________ |
| Periodo de muestreo | 20 ms | ___________ |
| Timeout comunicación | 500 ms | ___________ |

### 2.2 Configuración IMU MPU6050

| Parámetro | Valor |
|-----------|-------|
| Rango acelerómetro | ±8 G |
| Rango giroscopio | ±500 °/s |
| Filtro paso bajo | 21 Hz |

### 2.3 Umbrales de Control

| Umbral | Valor | Propósito |
|--------|-------|-----------|
| AccelZ para "mano arriba" | > 8.0 m/s² | Activar Servo2 |
| AccelZ para "mano abajo" | < 2.0 m/s² | Activar Servo1 |

---

## 3. Análisis de Señales del Sensor

### 3.1 Características de la Señal (DAC GPIO17)

Conectar osciloscopio y registrar:

| Condición | Amplitud (V) | Frecuencia dominante (Hz) | Ruido pico-pico (mV) | Observaciones |
|-----------|--------------|---------------------------|----------------------|---------------|
| Reposo | _____ | _____ | _____ | ___________ |
| Movimiento lento | _____ | _____ | _____ | ___________ |
| Movimiento rápido | _____ | _____ | _____ | ___________ |
| Vibración | _____ | _____ | _____ | ___________ |

**Adjuntar capturas del osciloscopio**

### 3.2 Tipos de Ruido Identificados

- [ ] Ruido blanco (aleatorio)
- [ ] Ruido de cuantización
- [ ] Interferencia de 60Hz
- [ ] Deriva del sensor (drift)
- [ ] Otro: _______________

**Descripción detallada**:
```
_________________________________________________
_________________________________________________
_________________________________________________
```

---

## 4. Experimento de Reproducibilidad

### 4.1 Definición de Puntos de Prueba

Definir 5 puntos dentro del espacio de trabajo:

```
Punto 1: Centro del espacio (referencia)
Punto 2: Extremo frontal
Punto 3: Extremo lateral derecho
Punto 4: Extremo lateral izquierdo
Punto 5: Punto intermedio diagonal
```

### 4.2 Tabla de Resultados

| Punto | Posición Esperada (cm) | Posición Medida (cm) | Posición Estimada (cm) |
|-------|------------------------|----------------------|------------------------|
|       | X | Y | X | Y | X | Y |
| **1** | _____ | _____ | _____ | _____ | _____ | _____ |
| **2** | _____ | _____ | _____ | _____ | _____ | _____ |
| **3** | _____ | _____ | _____ | _____ | _____ | _____ |
| **4** | _____ | _____ | _____ | _____ | _____ | _____ |
| **5** | _____ | _____ | _____ | _____ | _____ | _____ |

### 4.3 Método de Medición

**Posición Esperada**: 
```
_________________________________________________
```

**Posición Medida**: 
```
_________________________________________________
```

**Posición Estimada**: 
```
_________________________________________________
```

---

## 5. Análisis de Errores

### 5.1 Error entre Posición Esperada y Medida

| Punto | Error X (cm) | Error Y (cm) | Error Euclidiano (cm) |
|-------|--------------|--------------|------------------------|
| 1 | _____ | _____ | _____ |
| 2 | _____ | _____ | _____ |
| 3 | _____ | _____ | _____ |
| 4 | _____ | _____ | _____ |
| 5 | _____ | _____ | _____ |
| **Promedio** | _____ | _____ | _____ |
| **Desv. Est.** | _____ | _____ | _____ |

### 5.2 Error entre Posición Medida y Estimada

| Punto | Error X (cm) | Error Y (cm) | Error Euclidiano (cm) |
|-------|--------------|--------------|------------------------|
| 1 | _____ | _____ | _____ |
| 2 | _____ | _____ | _____ |
| 3 | _____ | _____ | _____ |
| 4 | _____ | _____ | _____ |
| 5 | _____ | _____ | _____ |
| **Promedio** | _____ | _____ | _____ |
| **Desv. Est.** | _____ | _____ | _____ |

---

## 6. Preguntas de Análisis

### 6.1 ¿Cuál de los errores es mayor? ¿Cuáles son las razones de tal resultado?

```
_________________________________________________
_________________________________________________
_________________________________________________
_________________________________________________
```

### 6.2 En promedio ¿Cuánto es el error entre la posición esperada y la medida? Explique las razones específicas.

```
Error promedio: _______ cm

Razones:
_________________________________________________
_________________________________________________
_________________________________________________
_________________________________________________
```

### 6.3 En promedio ¿Cuánto es el error entre la posición medida y la estimada? Explique las razones específicas.

```
Error promedio: _______ cm

Razones:
_________________________________________________
_________________________________________________
_________________________________________________
_________________________________________________
```

### 6.4 ¿Qué posibles soluciones el equipo puede plantear para minimizar los errores?

**Soluciones Propuestas**:

1. **Filtrado de señal**:
   ```
   _________________________________________________
   _________________________________________________
   ```

2. **Calibración**:
   ```
   _________________________________________________
   _________________________________________________
   ```

3. **Mecánica**:
   ```
   _________________________________________________
   _________________________________________________
   ```

4. **Control**:
   ```
   _________________________________________________
   _________________________________________________
   ```

5. **Otra**:
   ```
   _________________________________________________
   _________________________________________________
   ```

---

## 7. Observaciones Adicionales

### 7.1 Comportamiento del Sistema

**Sincronización**:
```
_________________________________________________
_________________________________________________
```

**Estabilidad**:
```
_________________________________________________
_________________________________________________
```

**Tiempo de respuesta**:
```
_________________________________________________
_________________________________________________
```

### 7.2 Problemas Encontrados

| Problema | Causa | Solución Aplicada |
|----------|-------|-------------------|
| _________ | _________ | _________ |
| _________ | _________ | _________ |
| _________ | _________ | _________ |

### 7.3 Características del Ruido

**Efecto en el control**:
- [ ] Movimientos bruscos
- [ ] Oscilación constante
- [ ] Deriva lenta
- [ ] Pérdida de precisión
- [ ] Otro: _______________

**Descripción detallada**:
```
_________________________________________________
_________________________________________________
_________________________________________________
```

---

## 8. Conclusiones Sprint 1

### 8.1 Logros

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

### 8.2 Limitaciones

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

### 8.3 Aprendizajes

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

### 8.4 Recomendaciones para Sprint 2

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

---

## 9. Anexos

### 9.1 Imágenes del Montaje

- [ ] Foto del guante con IMU
- [ ] Foto del brazo robótico
- [ ] Foto del circuito completo
- [ ] Capturas del osciloscopio

### 9.2 Gráficas

- [ ] Gráfica de señal cruda del DAC
- [ ] Gráfica de trayectoria esperada vs real
- [ ] Gráfica de errores por punto
- [ ] Gráfica de FFT de la señal

### 9.3 Código Fuente

- [x] Transmisor_Guante.ino
- [x] Receptor_Brazo.ino
- [ ] Scripts de análisis (Python/MATLAB)

---

**Firma de los integrantes**:

________________  ________________  ________________

**Fecha de entrega**: _______________

---

**Nota**: Este documento debe ser completado durante y después del experimento.  
Guardar todas las evidencias (fotos, capturas, datos crudos) para el informe final.
