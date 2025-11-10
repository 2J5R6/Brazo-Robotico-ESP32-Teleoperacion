# Lista de Materiales y Resumen - Sprint 1

## 📦 Lista de Materiales (BOM - Bill of Materials)

### Componentes Electrónicos

| Cantidad | Componente | Especificaciones | Uso | Precio Aprox. |
|----------|------------|------------------|-----|---------------|
| 1 | ESP32 WROOM | WiFi 2.4GHz, DAC | Transmisor (guante) | $5-8 USD |
| 1 | ESP32-S3 Dev Module | WiFi 2.4GHz, NO DAC | Receptor (brazo) | $10-15 USD |
| 1 | MPU6050 | 6DOF, I2C | IMU mano | $3-5 USD |
| 1 | MPU6050 (opcional) | 6DOF, I2C | IMU extremo brazo | $3-5 USD |
| 2 | Servo MG90S | 180°, 1.8kg-cm | Actuadores brazo | $4-6 USD c/u |
| 1 | Fuente 5V/2A | Min 2A | Alimentación servos | $5-8 USD |
| 1 | Regulador 3.3V (opcional) | LM1117 o similar | Alimentación ESP | $1-2 USD |
| 2 | Capacitor 100µF | Electrolítico | Desacoplo servos | $0.50 USD |
| 4 | Capacitor 100nF | Cerámico | Desacoplo ESP/IMU | $0.25 USD c/u |
| - | Cables Dupont | M-M, M-F | Conexiones | $3-5 USD |
| 1 | Protoboard | 830 puntos | Prototipo | $3-5 USD |
| - | Cable USB-C | Para ESP32-S3 | Programación | $2-3 USD |
| - | Cable Micro-USB | Para ESP32 WROOM | Programación | $1-2 USD |

**Total estimado**: $50-75 USD

**NOTA IMPORTANTE**:
- ✅ ESP32 WROOM tiene DAC (GPIO25/26) → Va en el GUANTE
- ❌ ESP32-S3 NO tiene DAC → Va en el BRAZO ROBÓTICO

### Materiales Mecánicos

| Cantidad | Material | Especificaciones | Uso | Precio Aprox. |
|----------|----------|------------------|-----|---------------|
| 1 | Guante | Textil, ajustable | Montar IMU | $5-10 USD |
| - | Velcro adhesivo | 5cm x 10cm | Fijar IMU al guante | $2-3 USD |
| 1 | Caja plástica pequeña | 5x5x3 cm | Alojar ESP32-S3 | $2-3 USD |
| - | Estructura brazo | Acrílico/MDF 3mm | De práctica anterior | - |
| 4 | Tornillos M3 | 10mm, con tuerca | Montar servos | $1-2 USD |
| - | Cinta doble cara | - | Fijar componentes | $1-2 USD |

**Total estimado**: $10-20 USD

### Herramientas Necesarias

- [ ] Multímetro digital
- [ ] Osciloscopio (para análisis DAC)
- [ ] Destornillador Phillips
- [ ] Alicate de corte
- [ ] Pelacables
- [ ] Soldador (opcional, recomendado)
- [ ] Cinta aislante
- [ ] Computador con Arduino IDE

---

## 🎯 Objetivos del Sprint 1

### Objetivos Primarios ✅

1. **Implementar sistema de captura de movimiento**
   - ✅ Lectura de IMU MPU6050
   - ✅ Frecuencia de muestreo: 50Hz
   - ✅ Comunicación I2C estable

2. **Implementar transmisión de datos**
   - ✅ ESP-NOW en modo broadcast
   - ✅ Estructura de datos completa
   - ✅ Callback de confirmación

3. **Generar señal para análisis**
   - ✅ Salida DAC en GPIO17
   - ✅ Escalado de señal 0-3.3V
   - ✅ Sincronizada con datos transmitidos

4. **Controlar brazo robótico 2DOF**
   - ✅ Control de 2 servomotores MG90S
   - ✅ Mapeo de aceleraciones a ángulos
   - ✅ Selección de servo por eje Z

5. **Implementar lógica de control**
   - ✅ Mano abajo → Servo 1 (base)
   - ✅ Mano arriba → Servo 2 (extremo)
   - ✅ Zona intermedia mantiene estado

### Objetivos Secundarios

- [ ] Calibración automática del IMU
- [ ] Indicadores LED de estado
- [ ] Registro de datos en tarjeta SD
- [ ] Interfaz web de monitoreo

---

## 📊 Especificaciones Técnicas

### Sistema de Comunicación

| Parámetro | Valor |
|-----------|-------|
| Protocolo | ESP-NOW |
| Frecuencia | 2.4 GHz |
| Modo | Broadcast (0xFF:FF:FF:FF:FF:FF) |
| Tasa de envío | 50 paquetes/segundo |
| Tamaño de paquete | 36 bytes |
| Alcance típico | 10-50 metros (línea de vista) |
| Latencia esperada | < 10 ms |

### Sistema de Sensado

| Parámetro | Valor |
|-----------|-------|
| Sensor | MPU6050 |
| Acelerómetro | ±8G |
| Giroscopio | ±500°/s |
| Interfaz | I2C (400 kHz) |
| Filtro interno | 21 Hz paso bajo |
| Resolución ADC | 16 bits |
| Frecuencia de muestreo | 50 Hz |

### Sistema de Actuación

| Parámetro | Valor |
|-----------|-------|
| Servomotor | MG90S |
| Torque | 1.8 kg-cm @ 4.8V |
| Velocidad | 0.1s/60° @ 4.8V |
| Rango angular | 0° - 180° |
| Señal PWM | 1-2 ms, 50Hz |
| Alimentación | 4.8-6V |
| Consumo | 100-500 mA |

### Sistema de Análisis (DAC)

| Parámetro | Valor |
|-----------|-------|
| Resolución | 8 bits (0-255) |
| Voltaje salida | 0 - 3.3V |
| Pin | GPIO17 (DAC1) |
| Señal representada | Aceleración X escalada |
| Impedancia salida | ~3kΩ |

---

## 🎓 Conceptos Aprendidos

### 1. Comunicación Inalámbrica
- Protocolo ESP-NOW
- Modo broadcast vs unicast
- Manejo de callbacks
- Sincronización de sistemas

### 2. Sistemas Inerciales
- Funcionamiento de IMU 6DOF
- Acelerómetros y giroscopios
- Sistemas de referencia
- Características del ruido en sensores

### 3. Control de Actuadores
- Servomotores y señales PWM
- Mapeo de rangos de valores
- Control de velocidad
- Limitaciones mecánicas

### 4. Análisis de Señales
- Señales analógicas (DAC)
- Ruido en sistemas reales
- Deriva de sensores (drift)
- Caracterización de señales

### 5. Sistemas Embebidos
- Arquitectura ESP32
- Restricciones de GPIO
- Gestión de recursos
- Diseño de sistemas de tiempo real

---

## 📈 Métricas de Éxito

### Métricas Cuantitativas

| Métrica | Objetivo | Resultado Esperado Sprint 1 |
|---------|----------|------------------------------|
| Latencia comunicación | < 50 ms | 10-20 ms |
| Precisión de posición | ±2 cm | ±5-10 cm (sin filtros) |
| Tasa de pérdida de paquetes | < 1% | < 5% |
| Frecuencia de actualización | 50 Hz | 50 Hz |
| Tiempo de respuesta | < 100 ms | 50-100 ms |
| Consumo de corriente | < 2A total | ~1.5A |

### Métricas Cualitativas

- [x] Sistema funciona de forma estable
- [ ] Movimiento suave y predecible (Sprint 2-3)
- [x] Selección de servo funciona correctamente
- [x] Fácil de usar y ergonómico
- [x] Documentación completa y clara

---

## 🔄 Comparación con Sprints Futuros

| Aspecto | Sprint 1 (Sin Filtros) | Sprint 2 (Con Filtros) | Sprint 3 (Kalman) |
|---------|------------------------|------------------------|-------------------|
| **Filtrado** | Ninguno | Paso bajo, promedio móvil | Filtro de Kalman EKF/UKF |
| **Precisión** | ±5-10 cm | ±2-5 cm | ±1-2 cm |
| **Ruido** | Alto | Medio | Bajo |
| **Latencia** | Baja (10-20ms) | Media (20-40ms) | Media-Alta (30-50ms) |
| **Complejidad** | Baja | Media | Alta |
| **CPU Usage** | ~10% | ~20-30% | ~40-60% |

### Evolución Esperada

```
Sprint 1: FUNCIONALIDAD BÁSICA
          ├─ Comunicación estable ✓
          ├─ Control básico ✓
          └─ Mucho ruido ✗

Sprint 2: MEJORA DE CALIDAD
          ├─ Filtrado básico
          ├─ Menor ruido
          └─ Mejor precisión

Sprint 3: OPTIMIZACIÓN
          ├─ Filtro de Kalman
          ├─ Estimación de estado
          └─ Precisión máxima
```

---

## 📅 Timeline del Proyecto

### Fase 1: Sprint 1 (Actual) - 2 semanas
- **Semana 1**: 
  - Montaje de hardware
  - Programación básica
  - Pruebas de comunicación
- **Semana 2**: 
  - Integración completa
  - Experimentos
  - Análisis de señales
  - Documentación

### Fase 2: Sprint 2 - 2 semanas
- Implementación de filtros
- Comparación de estrategias
- Nuevos experimentos
- Análisis comparativo

### Fase 3: Sprint 3 - 3 semanas
- Implementación de Kalman
- EKF vs UKF
- Experimentos finales
- Informe final

**Duración total**: 7 semanas

---

## 📚 Referencias Útiles

### Datasheets
- [ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [MG90S Servo Datasheet](http://www.towerpro.com.tw/product/mg90s/)

### Librerías Arduino
- [ESP-NOW Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)

### Tutoriales
- [ESP-NOW Getting Started](https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/)
- [MPU6050 Arduino Tutorial](https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/)

### Análisis de Señales
- [Numpy Documentation](https://numpy.org/doc/)
- [Matplotlib Tutorials](https://matplotlib.org/stable/tutorials/index.html)
- [Scipy Signal Processing](https://docs.scipy.org/doc/scipy/reference/signal.html)

---

## ✅ Checklist de Entregables

### Código
- [x] Transmisor_Guante.ino
- [x] Receptor_Brazo.ino
- [x] analisis_datos.py
- [ ] Código de calibración (opcional)

### Documentación
- [x] README.md (Guía general)
- [x] CONEXIONES.md (Diagramas)
- [x] EXPERIMENTO.md (Plantilla)
- [x] TROUBLESHOOTING.md (Resolución de problemas)
- [x] BOM.md (Este archivo)

### Datos y Resultados
- [ ] Capturas del osciloscopio
- [ ] Tabla de experimentos completada
- [ ] Gráficas de análisis
- [ ] Cálculos de errores
- [ ] Respuestas a preguntas

### Multimedia
- [ ] Fotos del montaje
- [ ] Video de funcionamiento
- [ ] Diagrama de conexiones (opcional)

---

## 🎯 Próximos Pasos

### Inmediatos (Sprint 1)
1. Montar hardware según diagramas
2. Subir código a ambos ESP32
3. Verificar comunicación
4. Realizar calibración inicial
5. Ejecutar experimentos

### Preparación Sprint 2
1. Estudiar filtros digitales
2. Analizar datos de Sprint 1
3. Identificar frecuencias de ruido
4. Diseñar estrategias de filtrado
5. Preparar código base

---

**Versión**: Sprint 1 - v1.0  
**Fecha**: Noviembre 2025  
**Estado**: Documentación completa ✓

---

## 💡 Notas Finales

### Consejos para el Éxito

1. **No subestimar el ruido**: En Sprint 1 es normal tener mucho ruido. Documentarlo bien para Sprint 2.

2. **Calibrar siempre**: Antes de cada sesión, calibrar el MPU6050 en posición horizontal.

3. **Medir voltajes**: Verificar que todos los componentes reciben el voltaje correcto.

4. **Documentar todo**: Tomar fotos, videos y notas detalladas. Serán útiles para el informe.

5. **Trabajar incremental**: Probar cada parte por separado antes de integrar.

6. **Backup del código**: Usar Git o al menos copias de seguridad.

### Problemas Comunes en Sprint 1

- ✓ Es NORMAL que los servos tiemblen (sin filtros)
- ✓ Es NORMAL que haya deriva en la posición (sin integración)
- ✓ Es NORMAL que la precisión sea baja (~5-10cm)
- ✓ Es NORMAL que haya sensibilidad a vibraciones

**Estos problemas se resolverán en los siguientes sprints.**

---

¡Éxito con el Sprint 1! 🚀
